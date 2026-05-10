//! Pure-Rust STL (stereolithography) ASCII + binary 3D mesh codec.
//!
//! Implements [`oxideav_mesh3d::Mesh3DDecoder`] +
//! [`oxideav_mesh3d::Mesh3DEncoder`] for both encodings of the STL
//! format defined by the 3D Systems *StereoLithography Interface
//! Specification* (October 1989).
//!
//! Reference: `docs/3d/stl/fabbers-stl-format.html` — Marshall Burns'
//! transcription of §6.5 of *Automated Fabrication: Improving
//! Productivity in Manufacturing* (Prentice-Hall, 1993).
//!
//! ## Coordinate convention
//!
//! STL has no unit field and no documented coordinate convention
//! beyond "all-positive octant" (an SLA-era artefact). Modern slicers
//! and authoring tools treat STL as Z-up + millimetres, which matches
//! the prevailing additive-manufacturing toolchain. We surface that
//! convention on the decoded [`Scene3D`] via
//! [`Axis::PosZ`](oxideav_mesh3d::Axis::PosZ) and
//! [`Unit::Millimetres`](oxideav_mesh3d::Unit::Millimetres);
//! downstream consumers that prefer glTF defaults (Y-up, metres) read
//! these fields and apply their own re-orientation rather than us
//! mutating geometry.
//!
//! ## Standalone build
//!
//! `oxideav-core` is gated behind the default-on `registry` cargo
//! feature. Drop the framework dependency entirely with:
//!
//! ```toml
//! oxideav-stl = { version = "0.0", default-features = false }
//! ```
//!
//! The encoder + decoder API stays available; only the `register()`
//! plumbing and the `oxideav-core` error alias disappear.

#![forbid(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod ascii;
pub mod binary;
pub mod color;
pub mod decoder;
pub mod encoder;
pub(crate) mod materialise_header;
#[cfg(feature = "trace")]
pub mod trace;

pub use color::{detect as detect_color_convention, ColorConvention, Stl16BitColor};
pub use decoder::StlDecoder;
pub use encoder::StlEncoder;

/// Format-id string used in the [`Mesh3DRegistry`](oxideav_mesh3d::Mesh3DRegistry).
pub const FORMAT_ID: &str = "stl";

/// File extensions handled by the STL codec.
pub const EXTENSIONS: &[&str] = &["stl"];

/// Wire `oxideav-stl` into a [`Mesh3DRegistry`](oxideav_mesh3d::Mesh3DRegistry).
///
/// Registers a decoder factory + encoder factory under format id
/// [`FORMAT_ID`] and extension [`EXTENSIONS`]. The encoder factory
/// produces a binary-mode encoder (the most common STL flavour); for
/// ASCII output, construct [`StlEncoder::new_ascii`] directly.
#[cfg(feature = "registry")]
pub fn register(registry: &mut oxideav_mesh3d::Mesh3DRegistry) {
    registry.register_decoder(
        FORMAT_ID,
        EXTENSIONS,
        Box::new(|| Box::new(StlDecoder::new())),
    );
    registry.register_encoder(
        FORMAT_ID,
        EXTENSIONS,
        Box::new(|| Box::new(StlEncoder::new_binary())),
    );
}
