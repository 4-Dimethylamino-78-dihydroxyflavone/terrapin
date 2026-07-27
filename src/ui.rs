//! Terminal output that survives Windows, and a window that stays open.
//!
//! Two things break a double-clicked console program on Windows 10, and both
//! of them make it look like the program itself is broken.
//!
//! The first is colour. Windows 10 conhost does not interpret ANSI escapes
//! unless a process explicitly asks it to, so `\x1b[1m` prints as the literal
//! characters `←[1m`. Constructing a `console::Term` is what performs that
//! request; where it cannot be granted — an old console, a redirected pipe, a
//! CI log — `colors_supported()` reports false and everything here degrades to
//! plain text rather than emitting bytes nothing will read.
//!
//! The second is that a process launched from Explorer owns its console
//! window, and Windows destroys that window the instant the process exits. The
//! error message is displayed correctly and for about four milliseconds. Every
//! path that can end a double-clicked run therefore has to pause first.

use console::{style, Term};

/// Ask the console for ANSI support, and fall back to plain text if refused.
pub fn init() {
    let term = Term::stdout();
    if !term.features().colors_supported() {
        console::set_colors_enabled(false);
        console::set_colors_enabled_stderr(false);
    }
}

pub fn bold(s: &str) -> String {
    style(s).bold().to_string()
}

pub fn cyan(s: &str) -> String {
    style(s).cyan().to_string()
}

pub fn yellow(s: &str) -> String {
    style(s).yellow().to_string()
}

pub fn red(s: &str) -> String {
    style(s).red().bold().to_string()
}

pub fn dim(s: &str) -> String {
    style(s).dim().to_string()
}

/// Hold the window open long enough to read what is on it.
///
/// Only when someone is actually looking: a redirected or piped run must not
/// block forever waiting for a keystroke that will never come.
pub fn pause() {
    if !console::user_attended() {
        return;
    }
    println!("\n{}", dim("Press Enter to close this window."));
    let mut sink = String::new();
    let _ = std::io::stdin().read_line(&mut sink);
}
