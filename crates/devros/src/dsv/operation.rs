//! DSV operation types

/// Operations supported in DSV files
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DsvOperation {
    /// Prepend value to environment variable (with deduplication)
    PrependNonDuplicate {
        variable: String,
        values: Vec<String>,
    },
    /// Prepend value if path exists (with deduplication)
    PrependNonDuplicateIfExists { variable: String, value: String },
    /// Append value to environment variable (with deduplication)
    AppendNonDuplicate { variable: String, value: String },
    /// Set environment variable to value
    Set { variable: String, value: String },
    /// Set environment variable only if not already set
    SetIfUnset { variable: String, value: String },
    /// Source another file
    Source { path: String },
}
