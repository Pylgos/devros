//! Shell type and shell command generation

use crate::{Error, Result};

/// Shell type for command generation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShellType {
    Bash,
    Zsh,
    Fish,
}

impl std::str::FromStr for ShellType {
    type Err = Error;

    fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "bash" => Ok(ShellType::Bash),
            "zsh" => Ok(ShellType::Zsh),
            "fish" => Ok(ShellType::Fish),
            _ => Err(Error::config(
                format!("Unknown shell type: {}", s),
                "Supported shells: bash, zsh, fish",
            )),
        }
    }
}

/// Escape a string for shell use
pub fn shell_escape(s: &str) -> String {
    // Use single quotes and escape any single quotes in the string
    format!("'{}'", s.replace('\'', "'\\''"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shell_escape() {
        assert_eq!(shell_escape("simple"), "'simple'");
        assert_eq!(shell_escape("with space"), "'with space'");
        assert_eq!(shell_escape("it's quoted"), "'it'\\''s quoted'");
    }
}
