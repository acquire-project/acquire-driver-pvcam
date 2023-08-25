//! Logger that forwards logs through to the acquire runtime.
//! Uses the `reporter` callback passed to the driver function.
use log::{Level, Metadata, Record};
use parking_lot::Mutex;
use std::ffi::{c_char, c_int, CString};

pub(crate) type ReporterCallback = Option<
    unsafe extern "C" fn(
        is_error: c_int,
        file: *const c_char,
        line: c_int,
        function: *const c_char,
        msg: *const c_char,
    ),
>;

pub(crate) struct AcquireLogger {
    reporter: Mutex<ReporterCallback>,
}

impl AcquireLogger {
    pub fn new(reporter: ReporterCallback) -> Self {
        Self {
            reporter: Mutex::new(reporter),
        }
    }
}

impl log::Log for AcquireLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // Don't filter based on log-level here. Just check if the reporter pointer looks right.
        self.reporter.lock().is_some()
    }

    /// Forwards logs via the Acquire runtime reporter callback.
    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let msg = format!("{}", record.args());

            let is_error = if record.level() == Level::Error { 1 } else { 0 };
            unsafe {
                // The lifetime of the strings doesn't need to escape this function.
                // `reporter()` is a callback provided by the acquire runtime. It assumes all strings
                // have the lifetime of the calling scope.
                let file =
                    CString::from_vec_unchecked(record.file().unwrap_or("(unknown file)").into());
                let place = CString::from_vec_unchecked(
                    record.module_path().unwrap_or("(unknown location)").into(),
                );
                let msg = CString::from_vec_unchecked(msg.into());
                self.reporter.lock().unwrap()(
                    is_error,
                    file.as_ptr(),
                    record.line().unwrap_or(0) as _,
                    place.as_ptr(),
                    msg.as_ptr(),
                );
            }
        }
    }

    /// Does nothing
    fn flush(&self) { /*no-op*/
    }
}
