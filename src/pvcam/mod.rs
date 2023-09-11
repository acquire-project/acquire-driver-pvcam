use std::fmt::{Display, Formatter};

pub(crate) mod api;
mod camera;
mod error;

pub(crate) use api::api;

#[derive(Default)]
struct Version {
    /// see pl_pvcam_get_ver() for version encoding
    inner: u16,
}

impl Display for Version {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let (major, minor, rev) = (self.inner >> 8, (self.inner & 0xff) >> 4, self.inner & 0xf);
        write!(f, "{major}.{minor}.{rev}")
    }
}
