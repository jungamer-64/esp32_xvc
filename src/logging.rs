//! Firmware logging policy.

#[cfg(feature = "xvc-log")]
macro_rules! xvc_log {
    ($($arg:tt)*) => {{
        esp_println::println!($($arg)*)
    }};
}

#[cfg(not(feature = "xvc-log"))]
macro_rules! xvc_log {
    ($($arg:tt)*) => {{
        if false {
            let _ = core::format_args!($($arg)*);
        }
    }};
}

pub(crate) use xvc_log;
