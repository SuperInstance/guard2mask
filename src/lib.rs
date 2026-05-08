//! GUARD-to-Mask Compiler
//!
//! Compiles GUARD DSL safety constraints to GDSII mask patterns for FLUX-LUCID hardware.
//!
//! Pipeline: GUARD source → parse → compile → CSP solve → GDSII generation
//!
//! # Example: Solving a CSP
//! ```rust
//! use guard2mask::{CSP, TernaryWeight, solve_csp};
//!
//! let mut csp = CSP::new();
//! csp.add_variable("throttle", vec![
//!     TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos,
//! ]);
//! csp.add_variable("engine_rpm", vec![
//!     TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos,
//! ]);
//!
//! csp.add_imply("throttle", TernaryWeight::Neg, "engine_rpm", TernaryWeight::Neg);
//!
//! if let Some(assignment) = solve_csp(&csp) {
//!     println!("Found solution!");
//! }
//! ```
//!
//! # Example: Generating GDSII
//! ```rust
//! use guard2mask::{generate_patterns, generate_gdsii, write_gdsii_file, Assignment, TernaryWeight};
//!
//! let mut assign = Assignment::new();
//! assign.values.insert("throttle".to_string(), TernaryWeight::Zero);
//! let patterns = generate_patterns(&assign);
//! let gdsii = generate_gdsii(&patterns);
//! // write_gdsii_file("output.gds", &gdsii);
//! ```

pub mod types;
pub mod parser;
pub mod solver;
pub mod via_gen;
pub mod compiler;

pub use types::*;
pub use parser::*;
pub use solver::*;
pub use via_gen::*;
pub use compiler::*;
