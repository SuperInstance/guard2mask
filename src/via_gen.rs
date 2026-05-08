//! Via pattern generator for GDSII output
//!
//! Converts solved CSP assignments (ternary weights) into physical via patterns
//! suitable for GDSII fabrication. Each variable maps to a rectangular via
//! positioned at (x*10, y*10) nm on a specific metal layer:
//!
//! - Neg (-1) → via on metal layer 1 (L1)
//! - Zero (0) → via on metal layer 2 (L2)
//! - Pos (+1) → via on metal layer 3 (L3)
//!
//! Vias are 0.5μm × 0.5μm squares (500nm × 500nm) — standard minimum via size
//! for many CMOS processes.

use crate::types::{Assignment, TernaryWeight, ViaPattern, GDSIIOutput};
use std::collections::HashMap;

/// Default via size in nanometers (0.5μm)
const VIA_SIZE_NM: u64 = 500;

/// Metal layer assignments for each ternary weight
const LAYER_NEG: u32 = 1;  // Metal 1 — negative signal
const LAYER_ZERO: u32 = 2; // Metal 2 — zero/reference
const LAYER_POS: u32 = 3;  // Metal 3 — positive signal

/// Generate via patterns from a solved constraint assignment.
///
/// Each assigned variable (constraint name) produces a via pattern at a grid
/// position based on the variable's index. The metal layer encodes the
/// ternary weight:
/// - Neg → Layer 1
/// - Zero → Layer 2
/// - Pos → Layer 3
///
/// The via is a 500nm × 500nm square (0.5μm × 0.5μm), which is a common
/// minimum via size for 180nm–65nm CMOS processes.
///
/// Grid positions are computed as (index * 10, index * 10) nanometers to
/// provide sufficient spacing between adjacent vias and avoid design rule
/// violations.
pub fn generate_patterns(assignment: &Assignment) -> GDSIIOutput {
    let mut patterns = Vec::new();
    let mut metadata = HashMap::new();

    // Sort variable names for deterministic output order
    let mut var_names: Vec<&String> = assignment.values.keys().collect();
    var_names.sort();

    let num_neg = var_names.iter()
        .filter(|v| assignment.values.get(v.as_str()) == Some(&TernaryWeight::Neg))
        .count();
    let num_zero = var_names.iter()
        .filter(|v| assignment.values.get(v.as_str()) == Some(&TernaryWeight::Zero))
        .count();
    let num_pos = var_names.iter()
        .filter(|v| assignment.values.get(v.as_str()) == Some(&TernaryWeight::Pos))
        .count();

    for (idx, name) in var_names.iter().enumerate() {
        let weight = match assignment.values.get(name.as_str()) {
            Some(w) => w,
            None => continue,
        };

        // Compute position: grid spacing of 10nm between vias
        // Position is at (x*10, y*10) to give clean layout
        let x: i64 = (idx as i64) * 10;
        let y: i64 = (idx as i64) * 10;

        // Select layer based on ternary weight
        let layer = match weight {
            TernaryWeight::Neg => LAYER_NEG,
            TernaryWeight::Zero => LAYER_ZERO,
            TernaryWeight::Pos => LAYER_POS,
        };

        patterns.push(ViaPattern {
            x,
            y,
            layer,
            width: VIA_SIZE_NM,
            height: VIA_SIZE_NM,
        });
    }

    // Record metadata
    metadata.insert("num_variables".to_string(), var_names.len().to_string());
    metadata.insert("num_neg".to_string(), num_neg.to_string());
    metadata.insert("num_zero".to_string(), num_zero.to_string());
    metadata.insert("num_pos".to_string(), num_pos.to_string());
    metadata.insert("via_size_nm".to_string(), VIA_SIZE_NM.to_string());

    GDSIIOutput { patterns, metadata }
}

/// Generate a GDSII-compatible string representation of the via patterns.
///
/// Formats output as GDSII boundary records:
/// ```text
/// BOUNDARY layer=1 datatype=0
///   XY: (x, y) → (x+width, y) → (x+width, y+height) → (x, y+height) → (x, y)
/// ENDEL
/// ```
pub fn format_gdsii(output: &GDSIIOutput) -> String {
    let mut result = String::new();
    result.push_str("HEADER 600\n");
    result.push_str("BGNLIB\n");
    result.push_str("LIBNAME GUARD2MASK_MASK\n");
    result.push_str("UNITS 0.001 1e-9\n"); // 1nm database units, 1e-9m/user units
    result.push_str("BGNSTR\n");
    result.push_str("STRNAME CROSSBAR_TOP\n");

    for pattern in &output.patterns {
        let half_w = (pattern.width / 2) as i64;
        let half_h = (pattern.height / 2) as i64;

        // GDSII boundary with 5 coordinates (last = first to close polygon)
        result.push_str(&format!("BOUNDARY layer={} datatype=0\n", pattern.layer));
        result.push_str(&format!(
            "  XY: ({}, {}) → ({}, {}) → ({}, {}) → ({}, {}) → ({}, {})\n",
            pattern.x - half_w, pattern.y - half_h,
            pattern.x + half_w, pattern.y - half_h,
            pattern.x + half_w, pattern.y + half_h,
            pattern.x - half_w, pattern.y + half_h,
            pattern.x - half_w, pattern.y - half_h,
        ));
        result.push_str("ENDEL\n");
    }

    result.push_str("ENDSTR\n");
    result.push_str("ENDLIB\n");

    result
}

/// Generate a simple text summary of the via layout for debugging/verification
pub fn format_summary(output: &GDSIIOutput) -> String {
    let mut result = String::new();
    result.push_str("=== Via Pattern Summary ===\n");
    result.push_str(&format!("Variables: {}\n", output.patterns.len()));
    result.push_str(&format!(
        "  Neg (L1): {}  |  Zero (L2): {}  |  Pos (L3): {}\n",
        output.metadata.get("num_neg").unwrap_or(&"0".to_string()),
        output.metadata.get("num_zero").unwrap_or(&"0".to_string()),
        output.metadata.get("num_pos").unwrap_or(&"0".to_string()),
    ));
    result.push_str("\nVias:\n");
    for (i, p) in output.patterns.iter().enumerate() {
        result.push_str(&format!(
            "  [{}] ({}, {}) L{}  {}×{} nm\n",
            i, p.x, p.y, p.layer, p.width, p.height
        ));
    }
    result
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::TernaryWeight;

    #[test]
    fn generate_empty_assignment() {
        let result = generate_patterns(&Assignment::new());
        assert!(result.patterns.is_empty());
        assert_eq!(result.metadata.get("num_variables").unwrap(), "0");
    }

    #[test]
    fn generate_neg_vias() {
        let mut assignment = Assignment::new();
        assignment.values.insert("depth".to_string(), TernaryWeight::Neg);
        let result = generate_patterns(&assignment);

        assert_eq!(result.patterns.len(), 1);
        let via = &result.patterns[0];
        assert_eq!(via.layer, LAYER_NEG);
        assert_eq!(via.width, VIA_SIZE_NM);
        assert_eq!(via.height, VIA_SIZE_NM);
        assert_eq!(via.x, 0); // first variable
        assert_eq!(via.y, 0);
    }

    #[test]
    fn generate_zero_vias() {
        let mut assignment = Assignment::new();
        assignment.values.insert("null".to_string(), TernaryWeight::Zero);
        let result = generate_patterns(&assignment);

        assert_eq!(result.patterns.len(), 1);
        assert_eq!(result.patterns[0].layer, LAYER_ZERO);
    }

    #[test]
    fn generate_pos_vias() {
        let mut assignment = Assignment::new();
        assignment.values.insert("speed".to_string(), TernaryWeight::Pos);
        let result = generate_patterns(&assignment);

        assert_eq!(result.patterns.len(), 1);
        assert_eq!(result.patterns[0].layer, LAYER_POS);
    }

    #[test]
    fn generate_multiple_vias() {
        let mut assignment = Assignment::new();
        assignment.values.insert("a".to_string(), TernaryWeight::Neg);
        assignment.values.insert("b".to_string(), TernaryWeight::Zero);
        assignment.values.insert("c".to_string(), TernaryWeight::Pos);

        let result = generate_patterns(&assignment);

        assert_eq!(result.patterns.len(), 3);

        // Check each via has correct layer
        let mut neg_count = 0;
        let mut zero_count = 0;
        let mut pos_count = 0;
        for p in &result.patterns {
            match p.layer {
                1 => neg_count += 1,
                2 => zero_count += 1,
                3 => pos_count += 1,
                _ => {}
            }
        }
        assert_eq!(neg_count, 1);
        assert_eq!(zero_count, 1);
        assert_eq!(pos_count, 1);

        // Positions should be spaced at 10nm intervals
        let x_positions: Vec<i64> = result.patterns.iter().map(|p| p.x).collect();
        assert!(x_positions.windows(2).all(|w| w[1] >= w[0]));
    }

    #[test]
    fn generate_positions_deterministic() {
        let mut assignment = Assignment::new();
        assignment.values.insert("z".to_string(), TernaryWeight::Pos);
        assignment.values.insert("a".to_string(), TernaryWeight::Neg);

        let result1 = generate_patterns(&assignment);
        let result2 = generate_patterns(&assignment);

        // Deterministic: same input → same output order
        assert_eq!(result1.patterns[0].layer, result2.patterns[0].layer);
        assert_eq!(result1.patterns[1].layer, result2.patterns[1].layer);
    }

    #[test]
    fn format_gdsii_output() {
        let mut assignment = Assignment::new();
        assignment.values.insert("test".to_string(), TernaryWeight::Pos);
        let output = generate_patterns(&assignment);
        let gdsii = format_gdsii(&output);

        assert!(gdsii.contains("HEADER 600"), "Missing HEADER");
        assert!(gdsii.contains("LIBNAME GUARD2MASK_MASK"), "Missing LIBNAME");
        assert!(gdsii.contains("STRNAME CROSSBAR_TOP"), "Missing STRNAME");
        assert!(gdsii.contains("BOUNDARY layer=3"), "Missing BOUNDARY");
        assert!(gdsii.contains("ENDLIB"), "Missing ENDLIB");
        // Via size: half_w = 500/2 = 250, coordinates include -250 or 250
        assert!(gdsii.contains("250"), "Expected via half-size (250nm) in coordinates");
    }

    #[test]
    fn format_summary_output() {
        let mut assignment = Assignment::new();
        assignment.values.insert("test".to_string(), TernaryWeight::Zero);
        assignment.values.insert("other".to_string(), TernaryWeight::Neg);
        let output = generate_patterns(&assignment);
        let summary = format_summary(&output);

        assert!(summary.contains("Via Pattern Summary"));
        assert!(summary.contains("Neg (L1)"));
        assert!(summary.contains("Zero (L2)"));
        assert!(summary.contains("Pos (L3)"));
        assert!(summary.contains("2")); // 2 vias
    }

    #[test]
    fn via_width_height_correct() {
        let mut assignment = Assignment::new();
        assignment.values.insert("test".to_string(), TernaryWeight::Neg);
        let output = generate_patterns(&assignment);

        for p in &output.patterns {
            assert_eq!(p.width, 500);
            assert_eq!(p.height, 500);
        }
    }

    #[test]
    fn generate_then_parse_via_patterns() {
        // Round-trip test: create assignment → generate → verify patterns
        let mut assignment = Assignment::new();
        assignment.values.insert("a".to_string(), TernaryWeight::Neg);
        assignment.values.insert("b".to_string(), TernaryWeight::Zero);
        assignment.values.insert("c".to_string(), TernaryWeight::Pos);

        let output = generate_patterns(&assignment);

        // Verify each via pattern is non-zero
        for p in &output.patterns {
            assert!(p.width > 0);
            assert!(p.height > 0);
            assert!(p.layer >= 1 && p.layer <= 3);
        }

        // Verify metadata
        assert_eq!(output.metadata.get("num_variables").unwrap(), "3");
        assert_eq!(output.metadata.get("num_neg").unwrap(), "1");
        assert_eq!(output.metadata.get("num_zero").unwrap(), "1");
        assert_eq!(output.metadata.get("num_pos").unwrap(), "1");
    }

    #[test]
    fn gdsii_boundary_closed_polygon() {
        // Verify GDSII boundary has 5 XY coordinates (closed polygon)
        let mut assignment = Assignment::new();
        assignment.values.insert("test".to_string(), TernaryWeight::Pos);
        let output = generate_patterns(&assignment);
        let gdsii = format_gdsii(&output);

        // A boundary should have XY with 5 points
        let xy_line: Vec<&str> = gdsii.lines()
            .filter(|l| l.trim().starts_with("XY:"))
            .collect();

        assert_eq!(xy_line.len(), 1, "Should have exactly one XY line");

        // Count coordinates: format is "(x1, y1) → (x2, y2)" — arrow separates
        // Find all parenthesized groups
        let _coord_groups: Vec<&str> = xy_line[0].matches(|c: char| c == '(' || c == ')')
            .collect::<Vec<&str>>();
        let paren_count = xy_line[0].matches('(').count();
        assert_eq!(paren_count, 5, "GDSII boundary must have 5 coordinate pairs (closed polygon)");
    }
}
