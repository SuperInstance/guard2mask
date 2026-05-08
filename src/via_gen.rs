//! Via pattern generator for GDSII output.
//!
//! Generates proper GDSII binary format with:
//! - Boundary box records (BOX) for each via
//! - Cell references (SREF) for repeated via patterns
//! - Library header/footer (LIBDIR, UNITS, ENDLIB)
//! - 500nm × 500nm vias on layers 1-3

use crate::types::*;

/// Default via size in database units (500nm at 1nm/DBU = 500 DBU)
const VIA_SIZE: u64 = 500;

/// Generate via patterns from a CSP assignment.
/// Maps each variable assignment to a via on a specific layer.
pub fn generate_patterns(assignment: &Assignment) -> GDSIIOutput {
    let mut output = GDSIIOutput::new();
    let mut x: i64 = 0;
    let mut y: i64 = 0;

    for (i, (_, val)) in assignment.values.iter().enumerate() {
        let layer = ((i as u32) % 3) + 1; // Alternate layers 1-3
        let offset = match val {
            TernaryWeight::Neg => -(VIA_SIZE as i64),     // left of center
            TernaryWeight::Zero => 0,                      // centered
            TernaryWeight::Pos => VIA_SIZE as i64,         // right of center
        };

        output.patterns.push(ViaPattern {
            x: x + offset,
            y,
            layer,
            width: VIA_SIZE,
            height: VIA_SIZE,
        });

        // Move to next column, wrap after 10 vias
        y += VIA_SIZE as i64 * 2;
        if y > VIA_SIZE as i64 * 20 {
            y = 0;
            x += VIA_SIZE as i64 * 6; // horizontal spacing between columns
        }
    }

    // Add metadata
    output.metadata.insert("via_size_nm".to_string(), VIA_SIZE.to_string());
    output.metadata.insert("total_vias".to_string(), assignment.values.len().to_string());
    output.metadata.insert("library".to_string(), "guard2mask_lib".to_string());

    output
}

/// Generate via patterns with custom via size
pub fn generate_patterns_with_size(assignment: &Assignment, via_size_nm: u64) -> GDSIIOutput {
    let mut output = GDSIIOutput::new();
    let mut x: i64 = 0;
    let mut y: i64 = 0;

    for (i, (_, val)) in assignment.values.iter().enumerate() {
        let layer = ((i as u32) % 3) + 1;
        let offset = match val {
            TernaryWeight::Neg => -(via_size_nm as i64),
            TernaryWeight::Zero => 0,
            TernaryWeight::Pos => via_size_nm as i64,
        };

        output.patterns.push(ViaPattern {
            x: x + offset,
            y,
            layer,
            width: via_size_nm,
            height: via_size_nm,
        });

        y += via_size_nm as i64 * 2;
        if y > via_size_nm as i64 * 20 {
            y = 0;
            x += via_size_nm as i64 * 6;
        }
    }

    output.metadata.insert("via_size_nm".to_string(), via_size_nm.to_string());
    output.metadata.insert("total_vias".to_string(), assignment.values.len().to_string());
    output.metadata.insert("library".to_string(), "guard2mask_lib".to_string());

    output
}

/// Generate a GDSII binary file from a solved assignment
pub fn generate_gdsii_file(assignment: &Assignment, path: &str) -> std::io::Result<()> {
    let patterns = generate_patterns(assignment);
    let gdsii = generate_gdsii(&patterns);
    write_gdsii_file(path, &gdsii)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generate_empty() {
        let result = generate_patterns(&Assignment::new());
        assert!(result.patterns.is_empty());
    }

    #[test]
    fn generate_single_via() {
        let mut assign = Assignment::new();
        assign.values.insert("throttle".to_string(), TernaryWeight::Zero);
        let result = generate_patterns(&assign);
        assert_eq!(result.patterns.len(), 1);
        let p = &result.patterns[0];
        assert_eq!(p.width, VIA_SIZE);
        assert_eq!(p.height, VIA_SIZE);
        assert_eq!(p.layer, 1);
        // Zero value should be centered
        assert_eq!(p.x, 0);
        assert_eq!(p.y, 0);
    }

    #[test]
    fn generate_three_vias_different_layers() {
        let mut assign = Assignment::new();
        assign.values.insert("a".to_string(), TernaryWeight::Neg);
        assign.values.insert("b".to_string(), TernaryWeight::Zero);
        assign.values.insert("c".to_string(), TernaryWeight::Pos);
        let result = generate_patterns(&assign);
        assert_eq!(result.patterns.len(), 3);
        // Layers should be 1, 2, 3
        for (i, p) in result.patterns.iter().enumerate() {
            assert_eq!(p.layer, ((i as u32) % 3) + 1);
        }
    }

    #[test]
    fn generate_gdsii_binary() {
        let mut assign = Assignment::new();
        assign.values.insert("throttle".to_string(), TernaryWeight::Neg);
        assign.values.insert("engine_rpm".to_string(), TernaryWeight::Zero);
        assign.values.insert("rudder".to_string(), TernaryWeight::Pos);
        let patterns = generate_patterns(&assign);
        let binary = generate_gdsii(&patterns);
        assert!(binary.len() > 100, "GDSII binary too short");
        // Verify ASCII strings are present
        let ascii = String::from_utf8_lossy(&binary);
        assert!(ascii.contains("guard2mask_lib"), "Missing library name");
        assert!(ascii.contains("VIA_LAYER_1"), "Missing VIA_LAYER_1");
        assert!(ascii.contains("TOP"), "Missing TOP structure");
    }

    #[test]
    fn generate_with_custom_size() {
        let mut assign = Assignment::new();
        assign.values.insert("x".to_string(), TernaryWeight::Pos);
        let result = generate_patterns_with_size(&assign, 1000);
        assert_eq!(result.patterns[0].width, 1000);
        assert_eq!(result.patterns[0].height, 1000);
        // Pos should be offset right
        assert_eq!(result.patterns[0].x, 1000);
    }
}
