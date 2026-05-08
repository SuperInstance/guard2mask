//! CSP solver with backtracking search
//!
//! Implements:
//! - AC-3 preprocessing for domain reduction
//! - Recursive backtracking with MRV heuristic
//! - Forward checking (arc consistency propagation)
//! - Conflict-directed backjumping
//!
//! The solver takes a `CSP` definition and produces an `Assignment`,
//! or returns `None` if no satisfying assignment exists.

use std::collections::{HashMap, HashSet};
use crate::types::*;
use crate::parser::{Constraint, Check};

/// Solve a CSP using full backtracking search with heuristics.
///
/// Returns `Some(Assignment)` if a satisfying assignment exists, `None` otherwise.
pub fn solve_csp(csp: &CSP) -> Option<Assignment> {
    // Phase 1: AC-3 preprocessing to reduce domains
    let mut current_domains = ac3_preprocess(csp);

    // If any domain becomes empty, problem is unsolvable
    for dom in current_domains.values() {
        if dom.is_empty() {
            return None;
        }
    }

    // Phase 2: Backtracking search with MRV and forward checking
    let mut assignment = Assignment::new();
    let (_conflict_set, result) = backtrack(
        csp,
        &mut current_domains,
        &mut assignment,
        0,
    );

    if result {
        Some(assignment)
    } else {
        None
    }
}

/// AC-3 arc consistency preprocessing.
///
/// Repeatedly removes values from domains that are inconsistent with binary constraints,
/// propagating domain reductions through the constraint graph.
fn ac3_preprocess(csp: &CSP) -> HashMap<String, Vec<TernaryWeight>> {
    let mut domains: HashMap<String, Vec<TernaryWeight>> = csp.domains.clone();

    // Build arc queue: all directed arcs (xi, xj) for each constraint
    let mut queue: Vec<(String, String)> = Vec::new();
    for c in &csp.constraints {
        match c {
            CSPConstraint::Imply { var1, var2, .. } => {
                queue.push((var1.clone(), var2.clone()));
                queue.push((var2.clone(), var1.clone()));
            }
            CSPConstraint::ForbidBoth { var1, var2, .. } => {
                queue.push((var1.clone(), var2.clone()));
                queue.push((var2.clone(), var1.clone()));
            }
        }
    }

    // Process arcs until queue is empty
    while let Some((xi, xj)) = queue.pop() {
        if revise(&mut domains, &xi, &xj, csp) {
            if domains.get(&xi).map_or(true, |d| d.is_empty()) {
                break; // domain wiped out — inconsistent
            }
            // Neighbors of xi (except xj) need re-checking
            for c in &csp.constraints {
                let vars = c.variables();
                for &v in &vars {
                    if v != xi && v != xj && vars.contains(&xi.as_str()) {
                        queue.push((v.to_string(), xi.clone()));
                    }
                }
            }
        }
    }

    domains
}

/// Revise domain of xi based on constraint with xj.
/// Returns true if domain of xi was changed.
fn revise(
    domains: &mut HashMap<String, Vec<TernaryWeight>>,
    xi: &str,
    xj: &str,
    csp: &CSP,
) -> bool {
    let mut changed = false;
    let values_xi = match domains.get(xi) {
        Some(d) => d.clone(),
        None => return false,
    };

    let values_xj = match domains.get(xj) {
        Some(d) => d.clone(),
        None => return false,
    };

    let mut new_domain: Vec<TernaryWeight> = Vec::new();

    for &vi in &values_xi {
        // Check if there exists some vj in domain(xj) satisfying all constraints
        let mut consistent = false;
        for &vj in &values_xj {
            if check_consistency_pair(xi, vi, xj, vj, csp) {
                consistent = true;
                break;
            }
        }
        if consistent {
            new_domain.push(vi);
        } else {
            changed = true;
        }
    }

    if changed {
        domains.insert(xi.to_string(), new_domain);
    }

    changed
}

/// Check if a pair of assignments is consistent with all constraints between xi and xj
fn check_consistency_pair(
    xi: &str,
    vi: TernaryWeight,
    xj: &str,
    vj: TernaryWeight,
    csp: &CSP,
) -> bool {
    for c in &csp.constraints {
        match c {
            CSPConstraint::Imply { var1, val1, var2, val2 } => {
                // If var1 == xi and val1 == vi, then var2 must equal val2
                if var1 == xi && *val1 == vi {
                    if var2 == xj && *val2 != vj {
                        return false;
                    }
                }
                // If var2 == xi and val2 == vi, then var1 must equal val1
                if var2 == xi && *val2 == vi {
                    if var1 == xj && *val1 != vj {
                        return false;
                    }
                }
            }
            CSPConstraint::ForbidBoth { var1, val1, var2, val2 } => {
                // Both assignments can't match the forbidden combination
                if (var1 == xi && *val1 == vi && var2 == xj && *val2 == vj)
                    || (var1 == xj && *val1 == vj && var2 == xi && *val2 == vi)
                {
                    return false;
                }
            }
        }
    }
    true
}

/// Recursive backtracking with MRV, forward checking, and conflict-directed backjumping.
///
/// Returns (conflict_set, success) where conflict_set is the set of variables
/// that caused failure (for backjumping).
fn backtrack(
    csp: &CSP,
    domains: &mut HashMap<String, Vec<TernaryWeight>>,
    assignment: &mut Assignment,
    _depth: usize,
) -> (HashSet<String>, bool) {
    // Check if all variables are assigned
    let unassigned: Vec<&str> = csp.variables.iter()
        .filter(|v| !assignment.values.contains_key(v.as_str()))
        .map(|v| v.as_str())
        .collect();

    if unassigned.is_empty() {
        // All variables assigned — check all constraints
        if check_all_constraints(csp, assignment) {
            return (HashSet::new(), true);
        } else {
            return (HashSet::new(), false);
        }
    }

    // MRV: pick the variable with smallest current domain
    let var = select_mrv_variable(csp, domains, &unassigned);

    // Get values in domain for this variable
    let domain_values = match domains.get(var) {
        Some(d) => d.clone(),
        None => return (HashSet::new(), false),
    };

    // Try each value in the domain
    let mut global_conflict_set: HashSet<String> = HashSet::new();

    for &value in &domain_values {
        // Assign variable
        assignment.values.insert(var.to_string(), value);

        // Forward checking: prune domains of unassigned variables
        let saved_domains = forward_check(csp, domains, assignment, var);

        // If forward checking didn't wipe out any domain, recurse with pruned domains
        let has_empty = saved_domains.iter().any(|(_, d)| d.is_empty());
        let (conflict_set, result) = if !has_empty {
            backtrack(csp, domains, assignment, _depth + 1)
        } else {
            // Forward checking found inconsistency — conflict set = current var
            let mut cs = HashSet::new();
            cs.insert(var.to_string());
            (cs, false)
        };

        // Restore domains to original state (undo forward checking)
        for (v, d) in &saved_domains {
            domains.insert(v.clone(), d.clone());
        }

        if result {
            return (HashSet::new(), true);
        }

        // Conflict-directed backjumping
        // The conflict set from the failure tells us which variables to jump to
        if conflict_set.contains(var) {
            // The current variable is in the conflict set — try next value
            global_conflict_set.extend(conflict_set.clone());
        } else if !conflict_set.is_empty() {
            // The conflict doesn't involve this variable — jump back
            assignment.values.remove(var);
            return (conflict_set, false);
        }

        // Undo assignment
        assignment.values.remove(var);
    }

    // All values tried and all failed
    if global_conflict_set.is_empty() {
        // No specific conflict — dead end
        let mut dead_end = HashSet::new();
        dead_end.insert(var.to_string());
        (dead_end, false)
    } else {
        // Return union of conflict sets minus current variable
        global_conflict_set.remove(var);
        (global_conflict_set, false)
    }
}

/// MRV heuristic: select the unassigned variable with the smallest current domain.
fn select_mrv_variable<'a>(
    _csp: &CSP,
    domains: &HashMap<String, Vec<TernaryWeight>>,
    unassigned: &[&'a str],
) -> &'a str {
    let mut best_var = unassigned[0];
    let mut best_size = usize::MAX;

    for &v in unassigned {
        if let Some(dom) = domains.get(v) {
            if dom.len() < best_size {
                best_size = dom.len();
                best_var = v;
            }
        }
    }

    best_var
}

/// Forward checking: after assigning `var`, prune domains of unassigned variables
/// that are connected to `var` by constraints.
///
/// Returns a map of changes made (variable -> new domain) so they can be restored.
fn forward_check(
    csp: &CSP,
    domains: &mut HashMap<String, Vec<TernaryWeight>>,
    assignment: &Assignment,
    var: &str,
) -> HashMap<String, Vec<TernaryWeight>> {
    let mut saved: HashMap<String, Vec<TernaryWeight>> = HashMap::new();

    let var_value = match assignment.values.get(var) {
        Some(v) => *v,
        None => return saved,
    };

    // Find all constraints involving `var`
    for c in &csp.constraints {
        match c {
            CSPConstraint::Imply { var1, val1, var2, val2 } => {
                // If var is var1 and assigned val1, then var2 must be val2
                if var1 == var && *val1 == var_value {
                    if let Some(dom) = domains.get(var2) {
                        if !assignment.values.contains_key(var2.as_str()) {
                            let new_dom: Vec<TernaryWeight> = dom.iter()
                                .filter(|&&v| v == *val2)
                                .copied()
                                .collect();
                            if new_dom.len() != dom.len() {
                                saved.insert(var2.clone(), dom.clone());
                                domains.insert(var2.clone(), new_dom);
                            }
                        }
                    }
                }
                // If var is var2 and assigned val2, then var1 must be val1
                if var2 == var && *val2 == var_value {
                    if let Some(dom) = domains.get(var1) {
                        if !assignment.values.contains_key(var1.as_str()) {
                            let new_dom: Vec<TernaryWeight> = dom.iter()
                                .filter(|&&v| v == *val1)
                                .copied()
                                .collect();
                            if new_dom.len() != dom.len() {
                                saved.insert(var1.clone(), dom.clone());
                                domains.insert(var1.clone(), new_dom);
                            }
                        }
                    }
                }
            }
            CSPConstraint::ForbidBoth { var1, val1, var2, val2 } => {
                // If var is var1 and assigned val1, then var2 cannot be val2
                if var1 == var && *val1 == var_value {
                    if let Some(dom) = domains.get(var2) {
                        if !assignment.values.contains_key(var2.as_str()) {
                            let new_dom: Vec<TernaryWeight> = dom.iter()
                                .filter(|&&v| v != *val2)
                                .copied()
                                .collect();
                            if new_dom.len() != dom.len() {
                                saved.insert(var2.clone(), dom.clone());
                                domains.insert(var2.clone(), new_dom);
                            }
                        }
                    }
                }
                // If var is var2 and assigned val2, then var1 cannot be val1
                if var2 == var && *val2 == var_value {
                    if let Some(dom) = domains.get(var1) {
                        if !assignment.values.contains_key(var1.as_str()) {
                            let new_dom: Vec<TernaryWeight> = dom.iter()
                                .filter(|&&v| v != *val1)
                                .copied()
                                .collect();
                            if new_dom.len() != dom.len() {
                                saved.insert(var1.clone(), dom.clone());
                                domains.insert(var1.clone(), new_dom);
                            }
                        }
                    }
                }
            }
        }
    }

    saved
}

/// Check all constraints against a full assignment
fn check_all_constraints(csp: &CSP, assignment: &Assignment) -> bool {
    for c in &csp.constraints {
        match c {
            CSPConstraint::Imply { var1, val1, var2, val2 } => {
                let v1 = assignment.values.get(var1);
                let v2 = assignment.values.get(var2);
                if let Some(&a1) = v1 {
                    if a1 == *val1 {
                        if let Some(&a2) = v2 {
                            if a2 != *val2 {
                                return false;
                            }
                        } else {
                            return false;
                        }
                    }
                }
            }
            CSPConstraint::ForbidBoth { var1, val1, var2, val2 } => {
                let v1 = assignment.values.get(var1);
                let v2 = assignment.values.get(var2);
                if let (Some(&a1), Some(&a2)) = (v1, v2) {
                    if a1 == *val1 && a2 == *val2 {
                        return false;
                    }
                }
            }
        }
    }
    true
}

/// Format the solver result as a human-readable string
pub fn format_assignment(assignment: &Assignment) -> String {
    let mut s = String::new();
    s.push_str("=== CSP Solution ===\n");
    for (var, val) in &assignment.values {
        s.push_str(&format!("  {} = {}\n", var, val.name()));
    }
    s
}

/// Legacy solver: compile GUARD constraints to assignment (stub with CSP bridge)
pub fn solve(constraints: &[Constraint]) -> Result<Assignment, String> {
    // Build a CSP from the constraints and solve it
    let mut csp = CSP::new();

    for c in constraints {
        for check in &c.checks {
            match check {
                Check::Range { start, end } => {
                    // Create variable for this range constraint
                    csp.add_variable(&format!("range_{}", c.name), vec![
                        TernaryWeight::Neg,
                        TernaryWeight::Zero,
                        TernaryWeight::Pos,
                    ]);
                    // Map range to ternary domain
                    if *start <= 0.0 && *end >= 0.0 {
                        // Variable can be Neg, Zero, or Pos depending on range
                        // For now, simplify: if range includes 0, prefer Zero
                    }
                }
                _ => {}
            }
        }
    }

    // Run CSP solver
    match solve_csp(&csp) {
        Some(assignment) => Ok(assignment),
        None => Err("No satisfying assignment found".to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solve_basic_imply() {
        let mut csp = CSP::new();
        csp.add_variable("throttle", vec![TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos]);
        csp.add_variable("engine_rpm", vec![TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos]);

        // Imply: throttle = Neg => engine_rpm = Neg
        csp.add_imply("throttle", TernaryWeight::Neg, "engine_rpm", TernaryWeight::Neg);

        let result = solve_csp(&csp);
        assert!(result.is_some(), "CSP should be solvable");
        let a = result.unwrap();
        // If throttle is Neg, engine_rpm must be Neg
        if a.values.get("throttle") == Some(&TernaryWeight::Neg) {
            assert_eq!(a.values.get("engine_rpm"), Some(&TernaryWeight::Neg));
        }
    }

    #[test]
    fn solve_forbid_both() {
        let mut csp = CSP::new();
        csp.add_variable("a", vec![TernaryWeight::Neg, TernaryWeight::Pos]);
        csp.add_variable("b", vec![TernaryWeight::Neg, TernaryWeight::Pos]);

        csp.add_forbid_both("a", TernaryWeight::Pos, "b", TernaryWeight::Pos);

        let result = solve_csp(&csp);
        assert!(result.is_some(), "CSP should be solvable");
        let a = result.unwrap();
        // a and b can't both be Pos
        assert!(!(a.values.get("a") == Some(&TernaryWeight::Pos)
            && a.values.get("b") == Some(&TernaryWeight::Pos)));
    }

    #[test]
    fn solve_unsatisfiable() {
        let mut csp = CSP::new();
        csp.add_variable("x", vec![TernaryWeight::Zero]);
        csp.add_variable("y", vec![TernaryWeight::Pos]);

        csp.add_imply("x", TernaryWeight::Zero, "y", TernaryWeight::Neg);
        // y can never be Neg since domain is [Pos]

        let result = solve_csp(&csp);
        assert!(result.is_none(), "Unexpectedly solvable");
    }

    #[test]
    fn solve_throttle_controller() {
        let mut csp = CSP::new();
        csp.add_variable("throttle_position", vec![
            TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos,
        ]);
        csp.add_variable("engine_rpm", vec![
            TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos,
        ]);
        csp.add_variable("rudder_angle", vec![
            TernaryWeight::Neg, TernaryWeight::Zero, TernaryWeight::Pos,
        ]);

        csp.add_imply("throttle_position", TernaryWeight::Neg, "engine_rpm", TernaryWeight::Neg);
        csp.add_forbid_both("throttle_position", TernaryWeight::Pos, "engine_rpm", TernaryWeight::Neg);

        let result = solve_csp(&csp);
        assert!(result.is_some(), "Throttle CSP should be solvable");
    }

    #[test]
    fn test_ac3_preprocessing() {
        let mut csp = CSP::new();
        csp.add_variable("x", vec![TernaryWeight::Neg, TernaryWeight::Pos]);
        csp.add_variable("y", vec![TernaryWeight::Pos]);

        csp.add_imply("x", TernaryWeight::Neg, "y", TernaryWeight::Pos);

        let domains = ac3_preprocess(&csp);
        // x should still have both Neg and Pos (both work with y=Pos)
        assert_eq!(domains["x"].len(), 2);
        // y should still have Pos
        assert_eq!(domains["y"].len(), 1);
    }

    #[test]
    fn solve_empty_csp() {
        let csp = CSP::new();
        let result = solve_csp(&csp);
        assert!(result.is_some(), "Empty CSP should be solvable");
    }
}
