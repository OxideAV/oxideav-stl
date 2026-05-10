//! 16-bit per-face colour extensions.
//!
//! The 3D Systems STL spec sets the per-triangle `uint16` attribute
//! byte count to zero. Two vendor conventions repurpose that slot for
//! a packed RGB triple plus a "valid" bit:
//!
//! ## VisCAM / SolidView
//!
//! Bit layout, MSB to LSB:
//!
//! ```text
//!  15  | 14 13 12 11 10 |  9  8  7  6  5 |  4  3  2  1  0
//! valid|       R        |        G       |        B
//! ```
//!
//! `valid = 1` means "use the per-face triple"; `valid = 0` means
//! "use the per-object default".
//!
//! ## Materialise Magics
//!
//! Bit layout, MSB to LSB:
//!
//! ```text
//!  15  | 14 13 12 11 10 |  9  8  7  6  5 |  4  3  2  1  0
//! valid|       B        |        G       |        R
//! ```
//!
//! Materialise inverts the byte-order *and* the meaning of the valid
//! bit: `valid = 0` means "use the per-face triple"; `valid = 1`
//! means "use the per-object default specified in the
//! `COLOR=R G B A`/`MATERIAL=…` header comment". The mirroring of
//! the channel order means a `0x801F` slot is interpreted as bright
//! red under Materialise but bright blue under VisCAM, so the
//! convention has to be picked before the bytes are read.
//!
//! ## Detection heuristic
//!
//! When the decoder is asked to interpret per-face attribute bytes
//! ([`detect`]), it scans the slot population:
//!
//! - If every slot has `bit_15 == 0` and every non-zero slot has
//!   non-zero R/G/B-low channels, Materialise is more plausible
//!   (the spec uses `0` as "valid").
//! - If a substantial fraction of slots have `bit_15 == 1`, VisCAM
//!   is more plausible (the spec uses `1` as "valid").
//! - Otherwise, no convention is asserted; callers can supply one
//!   explicitly.
//!
//! This is a heuristic — neither convention has a magic-number
//! signature, so 100 % accuracy is impossible. Real-world pipelines
//! either know the source tool or expose a UI control; callers that
//! want to round-trip a specific convention should pass it through
//! [`Stl16BitColor::from_word`] / [`Stl16BitColor::to_word`] directly.

/// Which 16-bit per-face colour convention is encoded in the
/// `attribute_byte_count` slot of a binary STL triangle.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ColorConvention {
    /// VisCAM / SolidView: `[valid:1][R:5][G:5][B:5]`. `valid=1` →
    /// per-face triple is live.
    ViscamSolidView,
    /// Materialise Magics: `[valid:1][B:5][G:5][R:5]`. `valid=0` →
    /// per-face triple is live.
    Materialise,
}

impl ColorConvention {
    /// Stable string tag used in the `Primitive::extras
    /// ["stl:color_convention"]` round-trip slot.
    pub fn as_str(&self) -> &'static str {
        match self {
            ColorConvention::ViscamSolidView => "viscam",
            ColorConvention::Materialise => "materialise",
        }
    }
}

/// Inverse of [`ColorConvention::as_str`]. Unknown tags return `Err`.
impl std::str::FromStr for ColorConvention {
    type Err = ();
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "viscam" => Ok(ColorConvention::ViscamSolidView),
            "materialise" => Ok(ColorConvention::Materialise),
            _ => Err(()),
        }
    }
}

/// One decoded 16-bit per-face colour slot.
///
/// `r`, `g`, `b` are 5-bit channels in the inclusive range `0..=31`.
/// `valid` reflects the convention's "this triple is live" semantics
/// (already normalised — `valid == true` means "use this colour"
/// regardless of how the source convention encoded it).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Stl16BitColor {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub valid: bool,
}

impl Stl16BitColor {
    /// Decode a single 16-bit attribute slot using `convention`.
    ///
    /// The slot is the little-endian `u16` value at the end of each
    /// 50-byte triangle record (i.e. `u16::from_le_bytes([lo, hi])`).
    pub fn from_word(convention: ColorConvention, slot: u16) -> Self {
        let bit15 = (slot >> 15) & 1;
        match convention {
            ColorConvention::ViscamSolidView => {
                // [valid:1][R:5][G:5][B:5]
                let r = ((slot >> 10) & 0x1f) as u8;
                let g = ((slot >> 5) & 0x1f) as u8;
                let b = (slot & 0x1f) as u8;
                Self {
                    r,
                    g,
                    b,
                    valid: bit15 == 1,
                }
            }
            ColorConvention::Materialise => {
                // [valid:1][B:5][G:5][R:5]
                let b = ((slot >> 10) & 0x1f) as u8;
                let g = ((slot >> 5) & 0x1f) as u8;
                let r = (slot & 0x1f) as u8;
                // Materialise inverts the valid-bit polarity: 0 means
                // "this slot is live".
                Self {
                    r,
                    g,
                    b,
                    valid: bit15 == 0,
                }
            }
        }
    }

    /// Encode this triple back to a 16-bit attribute slot under
    /// `convention`. R/G/B values are masked to 5 bits.
    pub fn to_word(&self, convention: ColorConvention) -> u16 {
        let r = (self.r & 0x1f) as u16;
        let g = (self.g & 0x1f) as u16;
        let b = (self.b & 0x1f) as u16;
        match convention {
            ColorConvention::ViscamSolidView => {
                let valid_bit = if self.valid { 1u16 << 15 } else { 0 };
                valid_bit | (r << 10) | (g << 5) | b
            }
            ColorConvention::Materialise => {
                // Materialise: valid=0 means "live" → invert.
                let valid_bit = if self.valid { 0 } else { 1u16 << 15 };
                valid_bit | (b << 10) | (g << 5) | r
            }
        }
    }

    /// Convert the 5-bit channel to an 8-bit value in the `0..=255`
    /// range (linear scale, `(c << 3) | (c >> 2)` so fully-saturated
    /// 5-bit `0x1f` maps to `0xff`).
    pub fn r8(&self) -> u8 {
        scale5to8(self.r)
    }
    /// See [`r8`](Self::r8).
    pub fn g8(&self) -> u8 {
        scale5to8(self.g)
    }
    /// See [`r8`](Self::r8).
    pub fn b8(&self) -> u8 {
        scale5to8(self.b)
    }
}

fn scale5to8(c5: u8) -> u8 {
    let c = c5 & 0x1f;
    (c << 3) | (c >> 2)
}

/// Heuristic: pick the colour convention that best fits the supplied
/// per-face attribute bytes (lo/hi pairs in file order).
///
/// Returns `None` when the population is too sparse or too ambiguous
/// to make a confident pick — callers should treat that as
/// "no convention asserted" and either fall back to raw byte
/// round-trip or surface a UI choice.
pub fn detect(attribute_bytes: &[u8]) -> Option<ColorConvention> {
    if attribute_bytes.len() < 2 || attribute_bytes.len() % 2 != 0 {
        return None;
    }
    let mut nonzero = 0usize;
    let mut bit15_set = 0usize;
    for chunk in attribute_bytes.chunks_exact(2) {
        let slot = u16::from_le_bytes([chunk[0], chunk[1]]);
        if slot != 0 {
            nonzero += 1;
            if slot & 0x8000 != 0 {
                bit15_set += 1;
            }
        }
    }
    // Need at least one non-zero slot to make any call.
    if nonzero == 0 {
        return None;
    }
    // VisCAM convention sets bit 15 ON live entries; Materialise sets
    // it OFF. Use a 50 % threshold of non-zero slots to discriminate.
    let viscam_score = bit15_set;
    let materialise_score = nonzero - bit15_set;
    if viscam_score > materialise_score {
        Some(ColorConvention::ViscamSolidView)
    } else if materialise_score > viscam_score {
        Some(ColorConvention::Materialise)
    } else {
        // Exact tie — bail. Caller picks.
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn viscam_red_round_trip() {
        // valid=1, R=31, G=0, B=0 → bit15 + R<<10 = 0x8000 | (31<<10) = 0xfc00
        let c = Stl16BitColor {
            r: 31,
            g: 0,
            b: 0,
            valid: true,
        };
        let word = c.to_word(ColorConvention::ViscamSolidView);
        assert_eq!(word, 0xfc00);
        let decoded = Stl16BitColor::from_word(ColorConvention::ViscamSolidView, word);
        assert_eq!(decoded, c);
    }

    #[test]
    fn viscam_green_round_trip() {
        // valid=1, G=31 → 0x8000 | (31 << 5) = 0x83e0
        let c = Stl16BitColor {
            r: 0,
            g: 31,
            b: 0,
            valid: true,
        };
        let word = c.to_word(ColorConvention::ViscamSolidView);
        assert_eq!(word, 0x83e0);
        assert_eq!(
            Stl16BitColor::from_word(ColorConvention::ViscamSolidView, word),
            c
        );
    }

    #[test]
    fn viscam_blue_round_trip() {
        // valid=1, B=31 → 0x8000 | 31 = 0x801f
        let c = Stl16BitColor {
            r: 0,
            g: 0,
            b: 31,
            valid: true,
        };
        let word = c.to_word(ColorConvention::ViscamSolidView);
        assert_eq!(word, 0x801f);
        assert_eq!(
            Stl16BitColor::from_word(ColorConvention::ViscamSolidView, word),
            c
        );
    }

    #[test]
    fn materialise_red_round_trip() {
        // valid=true (live) → bit15 = 0; R in low 5 bits = 31 → 0x001f
        let c = Stl16BitColor {
            r: 31,
            g: 0,
            b: 0,
            valid: true,
        };
        let word = c.to_word(ColorConvention::Materialise);
        assert_eq!(word, 0x001f);
        let decoded = Stl16BitColor::from_word(ColorConvention::Materialise, word);
        assert_eq!(decoded, c);
    }

    #[test]
    fn materialise_blue_round_trip() {
        // valid=true (live) → bit15 = 0; B in bits 10-14 = 31 → 0x7c00
        let c = Stl16BitColor {
            r: 0,
            g: 0,
            b: 31,
            valid: true,
        };
        let word = c.to_word(ColorConvention::Materialise);
        assert_eq!(word, 0x7c00);
        let decoded = Stl16BitColor::from_word(ColorConvention::Materialise, word);
        assert_eq!(decoded, c);
    }

    #[test]
    fn materialise_invalid_means_use_default() {
        // valid=false (default) → bit15 = 1; channels = 0 → 0x8000
        let c = Stl16BitColor {
            r: 0,
            g: 0,
            b: 0,
            valid: false,
        };
        let word = c.to_word(ColorConvention::Materialise);
        assert_eq!(word, 0x8000);
        assert_eq!(
            Stl16BitColor::from_word(ColorConvention::Materialise, word),
            c
        );
    }

    #[test]
    fn channel_5_to_8_scaling_is_full_range() {
        assert_eq!(scale5to8(0), 0);
        assert_eq!(scale5to8(31), 255);
        // Linear-ish replication: 0b11111000 | 0b00000111 = 0xff
        assert_eq!(scale5to8(16), (16 << 3) | (16 >> 2)); // 0x84
    }

    #[test]
    fn detect_picks_viscam_when_bit15_dominant() {
        // Three slots with bit15 set, one without → viscam.
        let bytes = vec![
            // 0xfc00 (lo=0x00, hi=0xfc) — viscam red, valid
            0x00, 0xfc, // 0x83e0 — viscam green, valid
            0xe0, 0x83, // 0x801f — viscam blue, valid
            0x1f, 0x80, // 0x001f — materialise red, valid (no bit 15)
            0x1f, 0x00,
        ];
        assert_eq!(detect(&bytes), Some(ColorConvention::ViscamSolidView));
    }

    #[test]
    fn detect_picks_materialise_when_bit15_clear() {
        // All non-zero slots have bit 15 clear → materialise.
        let bytes = vec![
            0x1f, 0x00, // 0x001f
            0xe0, 0x03, // 0x03e0
            0x00, 0x7c, // 0x7c00
        ];
        assert_eq!(detect(&bytes), Some(ColorConvention::Materialise));
    }

    #[test]
    fn detect_returns_none_for_all_zero() {
        let bytes = vec![0u8; 8];
        assert_eq!(detect(&bytes), None);
    }

    #[test]
    fn detect_returns_none_for_exact_tie() {
        // One bit15-set, one bit15-clear → tie.
        let bytes = vec![0x00, 0x80, 0x1f, 0x00];
        assert_eq!(detect(&bytes), None);
    }

    #[test]
    fn detect_returns_none_for_truncated_input() {
        assert_eq!(detect(&[0xff]), None);
        assert_eq!(detect(&[]), None);
    }

    #[test]
    fn convention_str_round_trip() {
        use std::str::FromStr;
        for c in [
            ColorConvention::ViscamSolidView,
            ColorConvention::Materialise,
        ] {
            assert_eq!(ColorConvention::from_str(c.as_str()), Ok(c));
        }
        assert!(ColorConvention::from_str("nope").is_err());
    }
}
