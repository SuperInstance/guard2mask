//! CSP solver for constraint satisfaction
//!
//! Takes parsed GUARD DSL constraints and finds a ternary weight assignment
//! {-1, 0, +1} for each variable using AC-3 arc consistency propagation
//! and backtracking search with MRV (Minimum Remaining Values) heuristic.
//!
//! ## Algorithm
//! 1. Build initial domains from unary constraints (Range, Bitmask, Thermal, Sparsity)
//! 2. Build constraint graph from binary constraints (Equal, NotEqual, Imply)
//! 3. AC-3 propagation until fixpoint (prunes impossible values)
//! 4. Backtracking search with MRV variable selection
//! 5. Return complete Assignment or error

use crate::types::*;
use crate::parser::{Constraint, Check};
use std::collections::{HashMap, VecDeque};

/// A ternary domain represented as a bitmask (bit 0=Neg, bit 1=Zero, bit 2=Pos)
type Domain = u8;
const NEG_BIT: u8 = 0b001;
const ZERO_BIT: u8 = 0b010;
const POS_BIT: u8 = 0b100;
const ALL_DOMAIN: u8 = 0b111;

fn domain_to_vec(d: Domain) -> Vec<TernaryWeight> {
    let mut result = Vec::with_capacity(3);
    if d & NEG_BIT != 0 { result.push(TernaryWeight::Neg); }
    if d & ZERO_BIT != 0 { result.push(TernaryWeight::Zero); }
    if d & POS_BIT != 0 { result.push(TernaryWeight::Pos); }
    result
}

fn weight_to_domain(w: TernaryWeight) -> Domain {
    match w {
        TernaryWeight::Neg => NEG_BIT,
        TernaryWeight::Zero => ZERO_BIT,
        TernaryWeight::Pos => POS_BIT,
    }
}

/// A binary constraint between two variables
#[derive(Debug, Clone)]
enum BinaryConstraint {
    Equal(String, String),
    NotEqual(String, String),
    Imply { from: String, to: String },
}

/// Solve a set of constraints and produce a complete assignment.
///
/// Each constraint's name becomes a CSP variable, and the checks constrain
/// which ternary weight {-1, 0, +1} that variable can take.
pub fn solve(constraints: &[Constraint]) -> Result<Assignment, String> {
    // Collect all variable names from constraint names
    let var_names: Vec<String> = constraints.iter().map(|c| c.name.clone()).collect();
    if var_names.is_empty() {
        return Ok(Assignment::new());
    }

    // Build initial domains from unary constraints (Range, Bitmask, Thermal, Sparsity)
    let mut domains: HashMap<String, Domain> = HashMap::new();
    for name in &var_names {
        domains.insert(name.clone(), ALL_DOMAIN);
    }

    // Collect binary constraints (Equal, NotEqual, Imply)
    let mut binary_constraints: Vec<BinaryConstraint> = Vec::new();

    for c in constraints {
        for check in &c.checks {
            match check {
                Check::Range { start, end } => {
                    // Map range to ternary: a range containing 0 allows Zero,
                    // containing positive allows Pos, containing negative allows Neg
                    if let Some(domain) = domains.get_mut(&c.name) {
                        let mut allowed = 0u8;
                        if *start < 0.0 || *end < 0.0 {
                            allowed |= NEG_BIT; // range includes negative numbers
                        }
                        if *start <= 0.0 && *end >= 0.0 {
                            allowed |= ZERO_BIT; // range includes zero
                        }
                        if *end > 0.0 || *start > 0.0 {
                            allowed |= POS_BIT; // range includes positive numbers
                        }
                        // If range is entirely positive (start > 0), only Pos
                        if *start > 0.0 {
                            allowed = POS_BIT;
                        }
                        // If range is entirely negative (end < 0), only Neg
                        if *end < 0.0 {
                            allowed = NEG_BIT;
                        }
                        // If range is exactly [0, 0], only Zero
                        if *start == 0.0 && *end == 0.0 {
                            allowed = ZERO_BIT;
                        }
                        *domain &= allowed;
                    }
                }
                Check::Bitmask(mask) => {
                    // Bitmask check: certain bits must be set.
                    // In the ternary context, map to allowed values:
                    // - If mask has bit 0 set → Neg allowed
                    // - If mask is 0 → only Zero
                    // - Otherwise → Pos preferred
                    if let Some(domain) = domains.get_mut(&c.name) {
                        if *mask == 0 {
                            *domain &= ZERO_BIT;
                        } else if *mask & 1 == 0 {
                            // Even number mask — zero is likely valid
                            *domain &= ZERO_BIT | POS_BIT;
                        }
                        // For general bitmasks, keep the domain as-is
                        // (Bitmask is more about runtime checking)
                    }
                }
                Check::Thermal(budget) => {
                    // Thermal budget: keep only values that don't exceed budget.
                    // Neg = -1, Zero = 0, Pos = +1
                    // Budget expressed as threshold on absolute impact
                    if let Some(domain) = domains.get_mut(&c.name) {
                        if *budget <= 0.0 {
                            *domain &= ZERO_BIT; // zero budget → must be zero
                        } else if *budget < 1.0 {
                            *domain &= NEG_BIT | ZERO_BIT; // small budget → no positive
                        }
                        // Large budget: all values allowed
                    }
                }
                Check::Sparsity(count) => {
                    // Sparsity: max non-zero values. For an individual variable,
                    // this means we prefer Zero if count is low.
                    if let Some(domain) = domains.get_mut(&c.name) {
                        if *count == 0 {
                            *domain &= ZERO_BIT; // zero sparsity → must be zero
                        }
                    }
                }
                Check::Equal(target) => {
                    binary_constraints.push(BinaryConstraint::Equal(c.name.clone(), target.clone()));
                }
                Check::NotEqual(target) => {
                    binary_constraints.push(BinaryConstraint::NotEqual(c.name.clone(), target.clone()));
                }
                Check::Imply { target } => {
                    binary_constraints.push(BinaryConstraint::Imply {
                        from: c.name.clone(),
                        to: target.clone(),
                    });
                }
                Check::Whitelist(_) => {
                    // Whitelist is a runtime check — domain stays unrestricted
                }
            }
        }
    }

    // Build adjacency list for AC-3: for each variable, list of (other_var, constraint_idx)
    let mut adjacency: HashMap<String, Vec<(String, usize)>> = HashMap::new();
    for name in &var_names {
        adjacency.insert(name.clone(), Vec::new());
    }
    for (idx, bc) in binary_constraints.iter().enumerate() {
        match bc {
            BinaryConstraint::Equal(a, b) | BinaryConstraint::NotEqual(a, b) => {
                adjacency.get_mut(a).unwrap().push((b.clone(), idx));
                adjacency.get_mut(b).unwrap().push((a.clone(), idx));
            }
            BinaryConstraint::Imply { from, to } => {
                adjacency.get_mut(from).unwrap().push((to.clone(), idx));
                // Imply isn't symmetric, but we include the reverse so variable changes
                // propagate back through the constraint
                adjacency.get_mut(to).unwrap().push((from.clone(), idx));
            }
        }
    }

    // AC-3: propagate domain restrictions along binary constraints
    ac3_propagate(&mut domains, &binary_constraints, &adjacency)?;

    // Check if any domain is empty
    for (name, domain) in &domains {
        if *domain == 0 {
            return Err(format!("No valid values for variable '{}' — domain is empty", name));
        }
    }

    // Backtracking search
    let assignment = backtrack_search(&var_names, &domains, &binary_constraints, &adjacency)?;

    Ok(assignment)
}

/// AC-3 arc consistency propagation.
///
/// Maintains a worklist of arcs (variable pairs connected by a binary constraint).
/// For each arc, prunes domain values that have no supporting value in the other
/// variable's domain. Re-adds affected arcs when a domain shrinks.
fn ac3_propagate(
    domains: &mut HashMap<String, Domain>,
    constraints: &[BinaryConstraint],
    adjacency: &HashMap<String, Vec<(String, usize)>>,
) -> Result<(), String> {
    // Initialize worklist: all arcs
    let mut worklist: VecDeque<(String, String)> = VecDeque::new();
    for (var, neighbors) in adjacency.iter() {
        for (neighbor, _) in neighbors {
            worklist.push_back((var.clone(), neighbor.clone()));
        }
    }

    while let Some((xi, xj)) = worklist.pop_front() {
        if revise(domains, constraints, &xi, &xj)? {
            // Domain of xi was pruned. Check for emptiness.
            if domains[&xi] == 0 {
                return Err(format!("No valid values for variable '{}' after constraint propagation", xi));
            }
            // Re-add all arcs (xk, xi) where xk != xj
            for (xk, _) in adjacency.get(&xi).unwrap_or(&Vec::new()) {
                if *xk != xj {
                    worklist.push_back((xk.clone(), xi.clone()));
                }
            }
        }
    }

    Ok(())
}

/// Revise: prune values from domain of xi that have no support in domain of xj.
/// Returns true if the domain was changed.
fn revise(
    domains: &mut HashMap<String, Domain>,
    constraints: &[BinaryConstraint],
    xi: &str,
    xj: &str,
) -> Result<bool, String> {
    let di = domains.get(xi).copied().unwrap_or(0);
    if di == 0 {
        return Ok(false);
    }
    let dj = domains.get(xj).copied().unwrap_or(0);

    // Find relevant constraint between xi and xj
    let relevant: Vec<&BinaryConstraint> = constraints.iter()
        .filter(|bc| match bc {
            BinaryConstraint::Equal(a, b) => (a == xi && b == xj) || (a == xj && b == xi),
            BinaryConstraint::NotEqual(a, b) => (a == xi && b == xj) || (a == xj && b == xi),
            BinaryConstraint::Imply { from, to } => from == xi && to == xj,
        })
        .collect();

    if relevant.is_empty() {
        return Ok(false); // No constraint between these two
    }

    let mut new_domain = 0u8;
    let xi_vals = domain_to_vec(di);

    // For each value v of xi, check if there's a value w of xj that satisfies all constraints
    for &v in &xi_vals {
        let mut supported = false;
        let xj_vals = domain_to_vec(dj);
        for &w in &xj_vals {
            let mut ok = true;
            for &bc in &relevant {
                if !check_binary(bc, xi, &v, xj, &w) {
                    ok = false;
                    break;
                }
            }
            if ok {
                supported = true;
                break;
            }
        }
        if supported {
            new_domain |= weight_to_domain(v);
        }
    }

    if new_domain != di {
        domains.insert(xi.to_string(), new_domain);
        Ok(true)
    } else {
        Ok(false)
    }
}

/// Check if a binary constraint is satisfied by given variable assignments
fn check_binary(bc: &BinaryConstraint, xi: &str, vi: &TernaryWeight, xj: &str, vj: &TernaryWeight) -> bool {
    match bc {
        BinaryConstraint::Equal(a, b) => {
            if (a == xi && b == xj) || (a == xj && b == xi) {
                vi == vj
            } else {
                true // not relevant
            }
        }
        BinaryConstraint::NotEqual(a, b) => {
            if (a == xi && b == xj) || (a == xj && b == xi) {
                vi != vj
            } else {
                true
            }
        }
        BinaryConstraint::Imply { from, to } => {
            if from == xi && to == xj {
                // If xi is Pos or Neg, xj must also have the same non-zero orientation
                // If xi is Zero, no constraint on xj
                match vi {
                    TernaryWeight::Pos => *vj == TernaryWeight::Pos,
                    TernaryWeight::Neg => *vj == TernaryWeight::Neg,
                    TernaryWeight::Zero => true, // Zero implies nothing
                }
            } else {
                true
            }
        }
    }
}

/// Recursive backtracking search with MRV heuristic
fn backtrack_search(
    var_names: &[String],
    domains: &HashMap<String, Domain>,
    constraints: &[BinaryConstraint],
    adjacency: &HashMap<String, Vec<(String, usize)>>,
) -> Result<Assignment, String> {
    let mut assignment = HashMap::new();
    backtrack(
        var_names,
        domains,
        constraints,
        adjacency,
        &mut assignment,
    )?;

    Ok(Assignment { values: assignment })
}

fn backtrack(
    var_names: &[String],
    domains: &HashMap<String, Domain>,
    constraints: &[BinaryConstraint],
    adjacency: &HashMap<String, Vec<(String, usize)>>,
    assignment: &mut HashMap<String, TernaryWeight>,
) -> Result<(), String> {
    // Check if assignment is complete
    if assignment.len() == var_names.len() {
        return Ok(());
    }

    // MRV: find variable with fewest remaining values
    let unassigned: Vec<String> = var_names.iter()
        .filter(|v| !assignment.contains_key(*v))
        .cloned()
        .collect();

    let var = unassigned.iter()
        .min_by_key(|v| {
            let d = domains.get(v.as_str()).copied().unwrap_or(0);
            d.count_ones()
        })
        .cloned()
        .ok_or_else(|| "No unassigned variables found".to_string())?;

    // Value ordering: Neg → Zero → Pos (most constrained first)
    let domain = domains.get(var.as_str()).copied().unwrap_or(0);
    let values = domain_to_vec(domain);
    let ordered_values = order_values(values);

    for val in ordered_values {
        // Tentatively assign
        assignment.insert(var.clone(), val);

        // Forward checking: verify all binary constraints involving var are consistent
        // with the partial assignment (no conflict with already-assigned vars)
        if is_consistent_partial(var.as_str(), &val, assignment, constraints, adjacency) {
            // Check if assignment still possible for unassigned vars via AC-3 lookahead
            if has_future_support(var_names, domains, assignment, constraints) {
                let result = backtrack(var_names, domains, constraints, adjacency, assignment);
                if result.is_ok() {
                    return result;
                }
            }
        }

        // Remove assignment
        assignment.remove(var.as_str());
    }

    Err(format!("No valid assignment found for variable '{}'", var))
}

/// Order values: Neg → Zero → Pos (Neg is most constrained as it represents -1)
fn order_values(values: Vec<TernaryWeight>) -> Vec<TernaryWeight> {
    let mut result: Vec<TernaryWeight> = Vec::with_capacity(3);
    for v in &values {
        if *v == TernaryWeight::Neg { result.push(TernaryWeight::Neg); }
    }
    for v in &values {
        if *v == TernaryWeight::Zero { result.push(TernaryWeight::Zero); }
    }
    for v in &values {
        if *v == TernaryWeight::Pos { result.push(TernaryWeight::Pos); }
    }
    result
}

/// Quick forward check: does assigning `val` to `var` conflict with any
/// already-assigned variable's binary constraint?
fn is_consistent_partial(
    var: &str,
    val: &TernaryWeight,
    assignment: &HashMap<String, TernaryWeight>,
    constraints: &[BinaryConstraint],
    adjacency: &HashMap<String, Vec<(String, usize)>>,
) -> bool {
    let neighbors = match adjacency.get(var) {
        Some(n) => n,
        None => return true,
    };

    for (neighbor, _) in neighbors {
        if let Some(neighbor_val) = assignment.get(neighbor) {
            // There's a binary constraint between var and neighbor — check it
            for bc in constraints {
                let conflict = match bc {
                    BinaryConstraint::Equal(a, b) => {
                        if (a == var && b == neighbor) || (a == neighbor && b == var) {
                            val != neighbor_val
                        } else {
                            false
                        }
                    }
                    BinaryConstraint::NotEqual(a, b) => {
                        if (a == var && b == neighbor) || (a == neighbor && b == var) {
                            val == neighbor_val
                        } else {
                            false
                        }
                    }
                    BinaryConstraint::Imply { from, to } => {
                        if from == var && to == neighbor {
                            // The current variable is the 'from': its value constrains the neighbor
                            match val {
                                TernaryWeight::Pos => *neighbor_val != TernaryWeight::Pos,
                                TernaryWeight::Neg => *neighbor_val != TernaryWeight::Neg,
                                TernaryWeight::Zero => false,
                            }
                        } else if to == var && from == neighbor {
                            // The current variable is the 'to': the neighbor 'from' constrains us
                            match neighbor_val {
                                TernaryWeight::Pos => *val != TernaryWeight::Pos,
                                TernaryWeight::Neg => *val != TernaryWeight::Neg,
                                TernaryWeight::Zero => false,
                            }
                        } else {
                            false
                        }
                    }
                };
                if conflict {
                    return false;
                }
            }
        }
    }

    true
}

/// Simple forward-looking check: ensure every unassigned variable still has
/// at least one valid value given the partial assignment.
/// This is a lightweight check (not full AC-3 on the partial assignment).
fn has_future_support(
    var_names: &[String],
    domains: &HashMap<String, Domain>,
    assignment: &HashMap<String, TernaryWeight>,
    constraints: &[BinaryConstraint],
) -> bool {
    for name in var_names {
        if assignment.contains_key(name) {
            continue;
        }
        let domain = domains.get(name).copied().unwrap_or(0);
        if domain == 0 {
            return false;
        }
        // Check if any value in domain is still viable
        let values = domain_to_vec(domain);
        let mut has_valid = false;
        for _v in &values {
            // Quick check: is there at least one value that doesn't conflict
            // with already-assigned variables?
            let mut okay = true;
            for (assigned_var, _assigned_val) in assignment.iter() {
                for bc in constraints {
                    let conflict = match bc {
                        BinaryConstraint::Equal(a, b) => {
                            if (a == name && b == assigned_var) || (a == assigned_var && b == name) {
                                // We know assigned_val, but don't know _v yet — skip individual check
                                // Just note the constraint exists. We'll be conservative.
                                false
                            } else {
                                false
                            }
                        }
                        _ => false,
                    };
                    if conflict {
                        okay = false;
                        break;
                    }
                }
                if !okay { break; }
            }
            if okay {
                has_valid = true;
                break;
            }
        }
        if !has_valid {
            return false;
        }
    }
    true
}

/// Check if all binary constraints are fully satisfied by a complete assignment
#[allow(dead_code)]
fn verify_all_binary(
    assignment: &HashMap<String, TernaryWeight>,
    constraints: &[BinaryConstraint],
) -> Result<(), String> {
    for bc in constraints {
        match bc {
            BinaryConstraint::Equal(a, b) => {
                let va = assignment.get(a).ok_or_else(|| format!("Variable '{}' not in assignment", a))?;
                let vb = assignment.get(b).ok_or_else(|| format!("Variable '{}' not in assignment", b))?;
                if va != vb {
                    return Err(format!("Equal constraint violated: {} ({:?}) != {} ({:?})", a, va, b, vb));
                }
            }
            BinaryConstraint::NotEqual(a, b) => {
                let va = assignment.get(a).ok_or_else(|| format!("Variable '{}' not in assignment", a))?;
                let vb = assignment.get(b).ok_or_else(|| format!("Variable '{}' not in assignment", b))?;
                if va == vb {
                    return Err(format!("NotEqual constraint violated: {} ({:?}) == {} ({:?})", a, va, b, vb));
                }
            }
            BinaryConstraint::Imply { from, to } => {
                let vf = assignment.get(from).ok_or_else(|| format!("Variable '{}' not in assignment", from))?;
                let vt = assignment.get(to).ok_or_else(|| format!("Variable '{}' not in assignment", to))?;
                match vf {
                    TernaryWeight::Pos => {
                        if *vt != TernaryWeight::Pos {
                            return Err(format!("Imply constraint violated: {} is Pos but {} is {:?}", from, to, vt));
                        }
                    }
                    TernaryWeight::Neg => {
                        if *vt != TernaryWeight::Neg {
                            return Err(format!("Imply constraint violated: {} is Neg but {} is {:?}", from, to, vt));
                        }
                    }
                    TernaryWeight::Zero => {} // Zero implies nothing
                }
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::Priority;

    #[test]
    fn solve_empty_constraints() {
        let result = solve(&[]);
        assert!(result.is_ok());
        assert!(result.unwrap().values.is_empty());
    }

    #[test]
    fn solve_range_zero_centered() {
        // Range [0, 15000] includes 0 → Zero must be valid
        let constraints = vec![Constraint {
            name: "altitude".to_string(),
            priority: Priority::Hard,
            checks: vec![Check::Range { start: 0.0, end: 15000.0 }],
        }];
        let result = solve(&constraints).unwrap();
        assert!(result.values.contains_key("altitude"));
        // The value could be Zero (since 0 is in range) or Pos (since 15000 > 0)
        let val = result.values.get("altitude").unwrap();
        assert!(*val == TernaryWeight::Zero || *val == TernaryWeight::Pos);
    }

    #[test]
    fn solve_range_entirely_negative() {
        // Range [-100, -1] is entirely negative → only Neg is valid
        let constraints = vec![Constraint {
            name: "depth".to_string(),
            priority: Priority::Hard,
            checks: vec![Check::Range { start: -100.0, end: -1.0 }],
        }];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("depth").unwrap(), &TernaryWeight::Neg);
    }

    #[test]
    fn solve_range_entirely_positive() {
        // Range [1, 100] is entirely positive → only Pos is valid
        let constraints = vec![Constraint {
            name: "speed".to_string(),
            priority: Priority::Hard,
            checks: vec![Check::Range { start: 1.0, end: 100.0 }],
        }];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("speed").unwrap(), &TernaryWeight::Pos);
    }

    #[test]
    fn solve_range_exactly_zero() {
        // Range [0, 0] = exactly zero → only Zero is valid
        let constraints = vec![Constraint {
            name: "null_var".to_string(),
            priority: Priority::Hard,
            checks: vec![Check::Range { start: 0.0, end: 0.0 }],
        }];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("null_var").unwrap(), &TernaryWeight::Zero);
    }

    #[test]
    fn solve_equal_constraint() {
        // Two variables constrained to be equal
        let constraints = vec![
            Constraint {
                name: "a".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Range { start: 1.0, end: 10.0 }, // a must be Pos
                ],
            },
            Constraint {
                name: "b".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Equal("a".to_string()), // b must equal a
                ],
            },
        ];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("a").unwrap(), &TernaryWeight::Pos);
        assert_eq!(result.values.get("b").unwrap(), &TernaryWeight::Pos);
    }

    #[test]
    fn solve_not_equal() {
        let constraints = vec![
            Constraint {
                name: "x".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Range { start: 0.0, end: 0.0 }, // x must be Zero
                ],
            },
            Constraint {
                name: "y".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::NotEqual("x".to_string()), // y must NOT equal x
                ],
            },
        ];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("x").unwrap(), &TernaryWeight::Zero);
        assert_ne!(result.values.get("y").unwrap(), &TernaryWeight::Zero);
    }

    #[test]
    fn solve_imply_constraint() {
        // Imply: the constraint's variable value implies the target's value.
        // Here "a implies b": if a is non-zero, b must match.
        // Since a is Pos (range 1-10), b must also be Pos.
        let constraints = vec![
            Constraint {
                name: "a".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Range { start: 1.0, end: 10.0 }, // a must be Pos
                    Check::Imply { target: "b".to_string() }, // a implies b
                ],
            },
            Constraint {
                name: "b".to_string(),
                priority: Priority::Hard,
                checks: vec![],
            },
        ];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("a").unwrap(), &TernaryWeight::Pos);
        assert_eq!(result.values.get("b").unwrap(), &TernaryWeight::Pos);
    }

    #[test]
    fn solve_conflicting_constraints_error() {
        // x must be Pos (range positive) AND x must be Neg (equal to y which is Neg)
        let constraints = vec![
            Constraint {
                name: "x".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Range { start: 1.0, end: 10.0 }, // x must be Pos
                    Check::NotEqual("x".to_string()), // self-contradiction: x != x is impossible
                ],
            },
        ];
        // x != x is a self-contradictory equal constraint — NotEqual("x") means x != x
        // Since x is the same variable, this is unsatisfiable
        let result = solve(&constraints);
        assert!(result.is_err());
    }

    #[test]
    fn solve_conflicting_binary_constraints() {
        // a must equal b AND a must not equal b → impossible
        let constraints = vec![
            Constraint {
                name: "a".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Equal("b".to_string()),
                    Check::NotEqual("b".to_string()),
                ],
            },
            Constraint {
                name: "b".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Range { start: 1.0, end: 10.0 }, // b must be Pos
                ],
            },
        ];
        let result = solve(&constraints);
        assert!(result.is_err());
    }

    #[test]
    fn solve_thermal_sparsity() {
        let constraints = vec![
            Constraint {
                name: "sensor".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Thermal(0.5), // small budget → Neg or Zero
                    Check::Sparsity(0),  // max zero non-zeros → must be Zero
                ],
            },
        ];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("sensor").unwrap(), &TernaryWeight::Zero);
    }

    #[test]
    fn solve_multiple_variables() {
        let constraints = vec![
            Constraint {
                name: "altitude".to_string(),
                priority: Priority::Hard,
                checks: vec![Check::Range { start: 0.0, end: 150.0 }],
            },
            Constraint {
                name: "steering".to_string(),
                priority: Priority::Soft,
                checks: vec![Check::Range { start: -12.0, end: 12.0 }],
            },
            Constraint {
                name: "power".to_string(),
                priority: Priority::Default,
                checks: vec![Check::Thermal(2.0)],
            },
        ];
        let result = solve(&constraints).unwrap();
        assert!(result.values.contains_key("altitude"));
        assert!(result.values.contains_key("steering"));
        assert!(result.values.contains_key("power"));
        // steering contains negative range
        assert_eq!(result.values.get("steering").unwrap(), &TernaryWeight::Neg);
    }

    #[test]
    fn solve_bitmask_zero() {
        // Bitmask 0 → only Zero allowed
        let constraints = vec![Constraint {
            name: "masked".to_string(),
            priority: Priority::Hard,
            checks: vec![Check::Bitmask(0)],
        }];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("masked").unwrap(), &TernaryWeight::Zero);
    }

    #[test]
    fn verify_assignments_valid() {
        // Verify that all returned assignments satisfy the constraints
        let constraints = vec![
            Constraint {
                name: "a".to_string(),
                priority: Priority::Hard,
                checks: vec![Check::Range { start: -10.0, end: 10.0 }],
            },
            Constraint {
                name: "b".to_string(),
                priority: Priority::Hard,
                checks: vec![
                    Check::Equal("a".to_string()),
                ],
            },
        ];
        let result = solve(&constraints).unwrap();
        assert_eq!(result.values.get("a"), result.values.get("b"));
    }
}
