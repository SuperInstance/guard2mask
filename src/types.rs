//! Core types for GUARD DSL, CSP solver, and mask compilation.

use std::collections::HashMap;

/// Ternary weight values: -1, 0, +1
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TernaryWeight {
    Neg,
    Zero,
    Pos,
}

impl TernaryWeight {
    pub fn name(&self) -> &str {
        match self {
            TernaryWeight::Neg => "Neg",
            TernaryWeight::Zero => "Zero",
            TernaryWeight::Pos => "Pos",
        }
    }
}

/// CSP variable assignment
#[derive(Debug, Clone, Default)]
pub struct Assignment {
    pub values: HashMap<String, TernaryWeight>,
}

impl Assignment {
    pub fn new() -> Self { Self::default() }
}

/// Via pattern for GDSII output
#[derive(Debug, Clone, PartialEq)]
pub struct ViaPattern {
    pub x: i64,
    pub y: i64,
    pub layer: u32,
    pub width: u64,
    pub height: u64,
}

/// GDSII output structure
#[derive(Debug, Clone, Default)]
pub struct GDSIIOutput {
    pub patterns: Vec<ViaPattern>,
    pub metadata: HashMap<String, String>,
}

impl GDSIIOutput {
    pub fn new() -> Self { Self::default() }
}

// ============================================================
// CSP (Constraint Satisfaction Problem) types
// ============================================================

/// CSP constraint types for the solver
#[derive(Debug, Clone)]
pub enum CSPConstraint {
    /// If var1 == val1, then var2 must be == val2
    Imply {
        var1: String,
        val1: TernaryWeight,
        var2: String,
        val2: TernaryWeight,
    },
    /// Forbid both var1 == val1 AND var2 == val2 simultaneously
    ForbidBoth {
        var1: String,
        val1: TernaryWeight,
        var2: String,
        val2: TernaryWeight,
    },
}

impl CSPConstraint {
    /// Get all variable names involved in this constraint
    pub fn variables(&self) -> Vec<&str> {
        match self {
            CSPConstraint::Imply { var1, var2, .. } => vec![var1.as_str(), var2.as_str()],
            CSPConstraint::ForbidBoth { var1, var2, .. } => vec![var1.as_str(), var2.as_str()],
        }
    }
}

/// A constraint satisfaction problem
#[derive(Debug, Clone)]
pub struct CSP {
    pub variables: Vec<String>,
    pub domains: HashMap<String, Vec<TernaryWeight>>,
    pub constraints: Vec<CSPConstraint>,
}

impl CSP {
    pub fn new() -> Self {
        CSP {
            variables: Vec::new(),
            domains: HashMap::new(),
            constraints: Vec::new(),
        }
    }

    /// Add a variable with its allowed domain values
    pub fn add_variable(&mut self, name: &str, domain: Vec<TernaryWeight>) {
        self.variables.push(name.to_string());
        self.domains.insert(name.to_string(), domain);
    }

    /// Add an implication constraint: if var1 == val1 then var2 == val2
    pub fn add_imply(&mut self, var1: &str, val1: TernaryWeight, var2: &str, val2: TernaryWeight) {
        self.constraints.push(CSPConstraint::Imply {
            var1: var1.to_string(),
            val1,
            var2: var2.to_string(),
            val2,
        });
    }

    /// Add a forbid-both constraint: forbid var1 == val1 AND var2 == val2 simultaneously
    pub fn add_forbid_both(&mut self, var1: &str, val1: TernaryWeight, var2: &str, val2: TernaryWeight) {
        self.constraints.push(CSPConstraint::ForbidBoth {
            var1: var1.to_string(),
            val1,
            var2: var2.to_string(),
            val2,
        });
    }
}

// ============================================================
// GDSII Binary Writer
// ============================================================

/// GDSII record data types
#[repr(u16)]
enum GDSIIDataType {
    NoData = 0,
    BitArray = 1,
    Int2 = 2,
    Int4 = 3,
    Real4 = 4,
    Real8 = 5,
    String = 6,
}

/// GDSII record types
#[repr(u16)]
enum GDSIIRecord {
    Header = 0x0002,
    BgnLib = 0x0102,
    LibName = 0x0206,
    Units = 0x0305,
    EndLib = 0x0400,
    BgnStr = 0x0502,
    StrName = 0x0606,
    EndStr = 0x0700,
    Boundary = 0x0800,
    Path = 0x0900,
    SRef = 0x0A00,
    ARef = 0x0B00,
    Text = 0x0C00,
    Layer = 0x0D02,
    DataType = 0x0E02,
    Width = 0x0F03,
    XY = 0x1003,
    EndEl = 0x1100,
    ColRow = 0x1302,
    BgnExt = 0x1402,
    EndExt = 0x1500,
}

/// Write a GDSII record to a byte buffer
fn write_record(buf: &mut Vec<u8>, rectype: u16, _data_type: u16, data: &[u8]) {
    let total_len = (4 + data.len()) as u16;
    buf.extend_from_slice(&total_len.to_be_bytes());
    buf.extend_from_slice(&rectype.to_be_bytes());
    buf.extend_from_slice(data);
    // Pad to even boundary
    if data.len() % 2 != 0 {
        buf.push(0);
    }
}

/// Write a 2-byte signed integer
fn write_int2(buf: &mut Vec<u8>, val: i16) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Write a 4-byte signed integer
fn write_int4(buf: &mut Vec<u8>, val: i32) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Write an 8-byte IEEE double (as GDSII real8)
fn write_real8(buf: &mut Vec<u8>, val: f64) {
    buf.extend_from_slice(&val.to_be_bytes());
}

/// Write an ASCII string, padded to even length with nulls
fn write_string(buf: &mut Vec<u8>, s: &str) {
    buf.extend_from_slice(s.as_bytes());
    if s.len() % 2 != 0 {
        buf.push(0);
    }
}

/// Generate binary GDSII from via patterns
pub fn generate_gdsii(output: &GDSIIOutput) -> Vec<u8> {
    let mut buf = Vec::new();

    // Library header
    write_record(&mut buf, GDSIIRecord::Header as u16, GDSIIDataType::Int2 as u16, &[0, 0x03, 0x00, 0x00]);
    write_record(&mut buf, GDSIIRecord::BgnLib as u16, GDSIIDataType::NoData as u16, &[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]); // bgnlib: 1980-01-01 00:00:00 mod/acc time
    write_record(&mut buf, GDSIIRecord::LibName as u16, GDSIIDataType::String as u16, b"guard2mask_lib\0");
    // Units: 1e-6 meters per DB unit, 1e-9 meters per user unit
    let mut units_data = Vec::new();
    write_real8(&mut units_data, 1e-6); // database units per meter
    write_real8(&mut units_data, 1e-9); // user units per meter
    write_record(&mut buf, GDSIIRecord::Units as u16, GDSIIDataType::Real8 as u16, &units_data);

    // Group vias by unique patterns for SREF
    let mut layer_map: HashMap<u32, Vec<&ViaPattern>> = HashMap::new();
    for p in &output.patterns {
        layer_map.entry(p.layer).or_default().push(p);
    }

    // For each layer, create a structure with the boundary boxes
    for (layer, patterns) in &layer_map {
        // Structure: LAYER_<N>
        let str_name = format!("VIA_LAYER_{}", layer);
        write_record(&mut buf, GDSIIRecord::BgnStr as u16, GDSIIDataType::NoData as u16, &[0x00; 8]);
        write_record(&mut buf, GDSIIRecord::StrName as u16, GDSIIDataType::String as u16, str_name.as_bytes());

        for p in patterns {
            write_boundary_box(&mut buf, *layer as i16, 0, p.x as i32 / 2, p.y as i32 / 2, p.width as u32, p.height as u32);
        }

        write_record(&mut buf, GDSIIRecord::EndStr as u16, GDSIIDataType::NoData as u16, &[]);
    }

    // Top-level structure placing all layer structures
    write_record(&mut buf, GDSIIRecord::BgnStr as u16, GDSIIDataType::NoData as u16, &[0x00; 8]);
    write_record(&mut buf, GDSIIRecord::StrName as u16, GDSIIDataType::String as u16, b"TOP\0");

    for layer in layer_map.keys() {
        let str_name = format!("VIA_LAYER_{}", layer);
        write_sref(&mut buf, &str_name, 0, 0);
    }

    write_record(&mut buf, GDSIIRecord::EndStr as u16, GDSIIDataType::NoData as u16, &[]);

    // End library
    write_record(&mut buf, GDSIIRecord::EndLib as u16, GDSIIDataType::NoData as u16, &[]);

    buf
}

/// Write a GDSII boundary box
fn write_boundary_box(buf: &mut Vec<u8>, layer: i16, datatype: i16, x: i32, y: i32, width: u32, height: u32) {
    write_record(buf, GDSIIRecord::Boundary as u16, GDSIIDataType::NoData as u16, &[]);
    let mut layer_data = Vec::new();
    write_int2(&mut layer_data, layer);
    write_record(buf, GDSIIRecord::Layer as u16, GDSIIDataType::Int2 as u16, &layer_data);
    let mut dt_data = Vec::new();
    write_int2(&mut dt_data, datatype);
    write_record(buf, GDSIIRecord::DataType as u16, GDSIIDataType::Int2 as u16, &dt_data);
    // XY data: 5 points (closed rectangle), each 4 bytes x and 4 bytes y
    let w = width as i32;
    let h = height as i32;
    let mut xy_data = Vec::with_capacity(5 * 8);
    // Bottom-left, bottom-right, top-right, top-left, back to bottom-left
    write_int4(&mut xy_data, x);
    write_int4(&mut xy_data, y);
    write_int4(&mut xy_data, x + w);
    write_int4(&mut xy_data, y);
    write_int4(&mut xy_data, x + w);
    write_int4(&mut xy_data, y + h);
    write_int4(&mut xy_data, x);
    write_int4(&mut xy_data, y + h);
    write_int4(&mut xy_data, x);
    write_int4(&mut xy_data, y);
    write_record(buf, GDSIIRecord::XY as u16, GDSIIDataType::Int4 as u16, &xy_data);
    write_record(buf, GDSIIRecord::EndEl as u16, GDSIIDataType::NoData as u16, &[]);
}

/// Write a GDSII structure reference (SREF)
fn write_sref(buf: &mut Vec<u8>, strname: &str, x: i32, y: i32) {
    write_record(buf, GDSIIRecord::SRef as u16, GDSIIDataType::NoData as u16, &[]);
    write_record(buf, GDSIIRecord::StrName as u16, GDSIIDataType::String as u16, strname.as_bytes());
    let mut xy_data = Vec::new();
    write_int4(&mut xy_data, x);
    write_int4(&mut xy_data, y);
    write_record(buf, GDSIIRecord::XY as u16, GDSIIDataType::Int4 as u16, &xy_data);
    write_record(buf, GDSIIRecord::EndEl as u16, GDSIIDataType::NoData as u16, &[]);
}

/// Write GDSII binary to file
pub fn write_gdsii_file(path: &str, data: &[u8]) -> std::io::Result<()> {
    std::fs::write(path, data)
}

/// Get a human-readable summary of a GDSII output
pub fn format_gdsii_summary(output: &GDSIIOutput) -> String {
    let mut s = String::new();
    s.push_str("=== GDSII Layout Summary ===\n");
    s.push_str(&format!("Total vias: {}\n", output.patterns.len()));
    for (i, p) in output.patterns.iter().enumerate() {
        s.push_str(&format!(
            "  Via {}: layer={}, pos=({},{}), size={}x{}\n",
            i, p.layer, p.x, p.y, p.width, p.height
        ));
    }
    if !output.metadata.is_empty() {
        s.push_str("Metadata:\n");
        for (k, v) in &output.metadata {
            s.push_str(&format!("  {}: {}\n", k, v));
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gdsii_empty_library() {
        let output = GDSIIOutput::new();
        let data = generate_gdsii(&output);
        // Should have header, bgnlib, libname, units, bgnstr, endstr, endlib at minimum
        assert!(data.len() > 40, "GDSII data too short: {} bytes", data.len());
        // First 2 bytes should be the length of header record (4 + 4 = 8)
        assert_eq!(data[0..2], [0x00, 0x08], "Header record length wrong");
    }

    #[test]
    fn gdsii_with_vias() {
        let mut output = GDSIIOutput::new();
        output.patterns.push(ViaPattern {
            x: 0, y: 0, layer: 1, width: 500, height: 500,
        });
        output.patterns.push(ViaPattern {
            x: 1000, y: 1000, layer: 2, width: 500, height: 500,
        });
        let data = generate_gdsii(&output);
        assert!(data.len() > 100, "GDSII data too short: {} bytes", data.len());
        // Should contain layer structures
        let ascii = String::from_utf8_lossy(&data);
        assert!(ascii.contains("VIA_LAYER_1"), "Missing VIA_LAYER_1");
        assert!(ascii.contains("VIA_LAYER_2"), "Missing VIA_LAYER_2");
        assert!(ascii.contains("TOP"), "Missing TOP structure");
    }
}
