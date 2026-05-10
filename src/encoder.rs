//! [`StlEncoder`] — [`Scene3D`]-in, bytes-out.
//!
//! Walks every mesh's `Triangles` primitives in scene-graph order and
//! concatenates them into a single STL output. Non-`Triangles`
//! topologies are rejected with [`Error::Unsupported`]. Per-face
//! normals are taken from the input primitive's `normals` field when
//! present and consistent; otherwise recomputed from positions via
//! the right-hand rule on each triangle's vertex order.
//!
//! Per-face attribute bytes survive a parse → reserialise round-trip
//! when present on `Mesh::extras["stl:per_face_attributes"]` as a hex
//! string (binary STL only — ASCII has no attribute slot).

use oxideav_mesh3d::{Error, Mesh3DEncoder, Result, Scene3D, Topology};

use crate::{ascii, binary};

/// Output flavour for [`StlEncoder`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StlFormat {
    /// Binary STL — 80-byte header + `uint32` triangle count + `N × 50`
    /// bytes per triangle. Default.
    Binary,
    /// ASCII STL — `solid … endsolid` token grammar.
    Ascii,
}

/// STL encoder — implements [`Mesh3DEncoder`].
#[derive(Debug)]
pub struct StlEncoder {
    format: StlFormat,
}

impl StlEncoder {
    /// Construct a binary-mode encoder.
    pub fn new_binary() -> Self {
        Self {
            format: StlFormat::Binary,
        }
    }

    /// Construct an ASCII-mode encoder.
    pub fn new_ascii() -> Self {
        Self {
            format: StlFormat::Ascii,
        }
    }

    /// Construct an encoder for the given `format`.
    pub fn new(format: StlFormat) -> Self {
        Self { format }
    }

    /// Output flavour this encoder will produce.
    pub fn format(&self) -> StlFormat {
        self.format
    }
}

impl Default for StlEncoder {
    fn default() -> Self {
        Self::new_binary()
    }
}

impl Mesh3DEncoder for StlEncoder {
    fn encode(&mut self, scene: &Scene3D) -> Result<Vec<u8>> {
        // STL is a single-mesh format; we walk every mesh in the scene
        // and emit one big triangle list. Reject non-Triangles
        // primitives up-front so the encoder side has a single contract.
        for mesh in &scene.meshes {
            for prim in &mesh.primitives {
                if prim.topology != Topology::Triangles {
                    return Err(Error::Unsupported(format!(
                        "STL only supports Triangles topology; got {:?}",
                        prim.topology
                    )));
                }
            }
        }
        match self.format {
            StlFormat::Binary => binary::encode(scene),
            StlFormat::Ascii => ascii::encode(scene),
        }
    }
}
