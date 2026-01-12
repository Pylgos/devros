//! DSV file parsing

use camino::Utf8PathBuf;

use crate::{Error, Result};

use super::operation::DsvOperation;

/// Parsed DSV file
#[derive(Debug, Clone)]
pub struct DsvFile {
    /// Path to the DSV file
    pub path: Utf8PathBuf,
    /// Operations defined in the file
    pub operations: Vec<DsvOperation>,
}

impl DsvFile {
    /// Parse a DSV file from disk
    pub fn parse(path: &camino::Utf8Path) -> Result<Self> {
        let content = std::fs::read_to_string(path)?;
        Self::parse_str(&content, path.to_path_buf())
    }

    /// Parse DSV content from a string
    pub fn parse_str(content: &str, path: Utf8PathBuf) -> Result<Self> {
        let mut operations = Vec::new();

        for (line_num, line) in content.lines().enumerate() {
            let line = line.trim();

            // Skip empty lines and comments
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            let parts: Vec<&str> = line.split(';').collect();
            if parts.is_empty() {
                continue;
            }

            let op_type = parts[0];
            match op_type {
                "prepend-non-duplicate" => {
                    if parts.len() < 2 {
                        return Err(Error::dsv_parse(
                            format!(
                                "Invalid prepend-non-duplicate at line {}: missing variable name",
                                line_num + 1
                            ),
                            "Format: prepend-non-duplicate;VARIABLE_NAME;value1;value2;...",
                        ));
                    }
                    let variable = parts[1].to_string();
                    let values: Vec<String> =
                        parts.iter().skip(2).map(|s| (*s).to_string()).collect();
                    operations.push(DsvOperation::PrependNonDuplicate { variable, values });
                }
                "prepend-non-duplicate-if-exists" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!(
                                "Invalid prepend-non-duplicate-if-exists at line {}",
                                line_num + 1
                            ),
                            "Format: prepend-non-duplicate-if-exists;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::PrependNonDuplicateIfExists {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "append-non-duplicate" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!("Invalid append-non-duplicate at line {}", line_num + 1),
                            "Format: append-non-duplicate;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::AppendNonDuplicate {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "set" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!("Invalid set at line {}", line_num + 1),
                            "Format: set;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::Set {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "set-if-unset" => {
                    if parts.len() < 3 {
                        return Err(Error::dsv_parse(
                            format!("Invalid set-if-unset at line {}", line_num + 1),
                            "Format: set-if-unset;VARIABLE_NAME;value",
                        ));
                    }
                    operations.push(DsvOperation::SetIfUnset {
                        variable: parts[1].to_string(),
                        value: parts[2].to_string(),
                    });
                }
                "source" => {
                    if parts.len() < 2 {
                        return Err(Error::dsv_parse(
                            format!("Invalid source at line {}", line_num + 1),
                            "Format: source;path/to/script",
                        ));
                    }
                    operations.push(DsvOperation::Source {
                        path: parts[1].to_string(),
                    });
                }
                _ => {
                    tracing::warn!(
                        line = line_num + 1,
                        operation = op_type,
                        "Unknown DSV operation"
                    );
                }
            }
        }

        Ok(DsvFile { path, operations })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_dsv_operations() {
        let content = r#"
# This is a comment
prepend-non-duplicate;PATH;bin
prepend-non-duplicate-if-exists;LD_LIBRARY_PATH;lib
append-non-duplicate;PYTHONPATH;site-packages
set;MY_VAR;value
set-if-unset;DEFAULT_VAR;default
source;other_package.dsv
"#;

        let dsv = DsvFile::parse_str(content, Utf8PathBuf::from("/test")).unwrap();

        assert_eq!(dsv.operations.len(), 6);

        assert!(matches!(
            &dsv.operations[0],
            DsvOperation::PrependNonDuplicate { variable, values }
            if variable == "PATH" && values == &["bin"]
        ));

        assert!(matches!(
            &dsv.operations[1],
            DsvOperation::PrependNonDuplicateIfExists { variable, value }
            if variable == "LD_LIBRARY_PATH" && value == "lib"
        ));

        assert!(matches!(
            &dsv.operations[2],
            DsvOperation::AppendNonDuplicate { variable, value }
            if variable == "PYTHONPATH" && value == "site-packages"
        ));

        assert!(matches!(
            &dsv.operations[3],
            DsvOperation::Set { variable, value }
            if variable == "MY_VAR" && value == "value"
        ));

        assert!(matches!(
            &dsv.operations[4],
            DsvOperation::SetIfUnset { variable, value }
            if variable == "DEFAULT_VAR" && value == "default"
        ));

        assert!(matches!(
            &dsv.operations[5],
            DsvOperation::Source { path }
            if path == "other_package.dsv"
        ));
    }
}
