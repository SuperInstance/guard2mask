//! Boat Throttle Controller — CSP Example
//!
//! A realistic constraint satisfaction problem for a boat's throttle controller:
//! - throttle_position: Neg (idle), Zero (half), Pos (full)
//! - engine_rpm: Neg (low), Zero (cruise), Pos (high)
//! - rudder_angle: Neg (port), Zero (center), Pos (starboard)
//!
//! Constraints:
//!   - Imply(throttle=Idle, engine_rpm=Low) — idle throttle means low RPM
//!   - NotEqual(throttle=Full, engine_rpm=Low) — full throttle can't have low RPM
//!   - Range for each variable (Neg, Zero, Pos)
//!
//! Usage:
//!   cargo run --example throttle_controller [-- --gdsii output.gds]

use guard2mask::{
    CSP, TernaryWeight, solve_csp, generate_patterns, generate_gdsii,
    write_gdsii_file, format_gdsii_summary, format_assignment,
};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let gdsii_output = if args.len() > 2 && args[1] == "--gdsii" {
        Some(args[2].clone())
    } else {
        None
    };

    println!("=== Boat Throttle Controller CSP ===");
    println!();

    // Build the CSP
    let mut csp = CSP::new();

    // Variable: throttle_position
    // Neg = idle, Zero = half, Pos = full
    csp.add_variable("throttle_position", vec![
        TernaryWeight::Neg,
        TernaryWeight::Zero,
        TernaryWeight::Pos,
    ]);

    // Variable: engine_rpm
    // Neg = low, Zero = cruise, Pos = high
    csp.add_variable("engine_rpm", vec![
        TernaryWeight::Neg,
        TernaryWeight::Zero,
        TernaryWeight::Pos,
    ]);

    // Variable: rudder_angle
    // Neg = port, Zero = center, Pos = starboard
    csp.add_variable("rudder_angle", vec![
        TernaryWeight::Neg,
        TernaryWeight::Zero,
        TernaryWeight::Pos,
    ]);

    // Constraint 1: Imply(throttle=Idle, engine_rpm=Low)
    // If throttle is idle (Neg), engine RPM must be low (Neg)
    csp.add_imply("throttle_position", TernaryWeight::Neg,
                  "engine_rpm", TernaryWeight::Neg);

    // Constraint 2: Forbid(throttle=Full, engine_rpm=Low)
    // Full throttle (Pos) cannot have low RPM (Neg)
    csp.add_forbid_both("throttle_position", TernaryWeight::Pos,
                        "engine_rpm", TernaryWeight::Neg);

    println!("Variables:");
    println!("  throttle_position ∈ {{Neg (idle), Zero (half), Pos (full)}}");
    println!("  engine_rpm        ∈ {{Neg (low), Zero (cruise), Pos (high)}}");
    println!("  rudder_angle      ∈ {{Neg (port), Zero (center), Pos (starboard)}}");
    println!();
    println!("Constraints:");
    println!("  Imply(throttle=Idle, engine_rpm=Low)");
    println!("  ForbidBoth(throttle=Full, engine_rpm=Low)");
    println!();

    // Solve the CSP
    println!("Solving...");
    match solve_csp(&csp) {
        Some(assignment) => {
            println!("\n{}", format_assignment(&assignment));

            // Print semantic interpretation
            println!("=== Interpretation ===");
            for (var, val) in &assignment.values {
                let desc = match (var.as_str(), val) {
                    ("throttle_position", TernaryWeight::Neg) => "throttle is idle (idle)",
                    ("throttle_position", TernaryWeight::Zero) => "throttle is half (cruise)",
                    ("throttle_position", TernaryWeight::Pos) => "throttle is full (full)",
                    ("engine_rpm", TernaryWeight::Neg) => "engine at low RPM",
                    ("engine_rpm", TernaryWeight::Zero) => "engine at cruise RPM",
                    ("engine_rpm", TernaryWeight::Pos) => "engine at high RPM",
                    ("rudder_angle", TernaryWeight::Neg) => "rudder turned port",
                    ("rudder_angle", TernaryWeight::Zero) => "rudder centered",
                    ("rudder_angle", TernaryWeight::Pos) => "rudder turned starboard",
                    _ => "unknown",
                };
                println!("  {}: {}", var, desc);
            }

            // Generate GDSII
            println!();
            let patterns = generate_patterns(&assignment);
            println!("{}", format_gdsii_summary(&patterns));

            if let Some(path) = gdsii_output {
                let gdsii = generate_gdsii(&patterns);
                match write_gdsii_file(&path, &gdsii) {
                    Ok(_) => println!("GDSII written to {}", path),
                    Err(e) => eprintln!("Error writing GDSII: {}", e),
                }
            }
        }
        None => {
            println!("No satisfying assignment found!");
            println!("The constraints are contradictory.");
        }
    }
}
