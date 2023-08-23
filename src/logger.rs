//! Logger that forwards logs through to the acquire runtime.
//! Uses the `reporter` callback passed to the driver function.
use log::{Level, Metadata, Record};
use std::ffi::{c_char, c_int};

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
    reporter: ReporterCallback,
}

impl AcquireLogger {
    pub fn new(reporter: ReporterCallback) -> Self {
        Self { reporter }
    }
}

impl log::Log for AcquireLogger {
    fn enabled(&self, _metadata: &Metadata) -> bool {
        // Don't filter based on log-level here. Just check if the reporter pointer looks right.
        self.reporter.is_some()
    }

    /// Forwards logs via the Acquire runtime reporter callback.
    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let msg = format!("{}", record.args());
            // The lifetime of the strings doesn't need to escape this function.
            // `reporter()` is a callback provided by the acquire runtime. It assumes all strings
            // have the lifetime of the calling scope.
            unsafe {
                // TODO: (nclack) test with a utf-8 string
                let is_error = if (record.level() == Level::Error) {
                    1
                } else {
                    0
                };
                self.reporter.unwrap()(
                    is_error,
                    record
                        .file()
                        .unwrap_or("(unknown file)")
                        .as_bytes()
                        .as_ptr() as _,
                    record.line().unwrap_or(0) as _,
                    record
                        .module_path()
                        .unwrap_or("(unknown module path")
                        .as_bytes()
                        .as_ptr() as _,
                    msg.as_str().as_bytes().as_ptr() as _,
                );
            }
        }
    }

    /// Does nothing
    fn flush(&self) { /*no-op*/
    }
}
