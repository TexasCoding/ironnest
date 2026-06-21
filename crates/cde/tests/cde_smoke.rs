// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Fork-locking end-to-end smoke test for `ironnest-cde`: drive the forked Importer → Layout →
//! CDEngine the way `lbf` does, and confirm the f64 flip preserves collision-detection behaviour.

use ironnest_cde::collision_detection::CDEConfig;
use ironnest_cde::entities::Layout;
use ironnest_cde::geometry::DTransformation;
use ironnest_cde::geometry::fail_fast::SPSurrogateConfig;
use ironnest_cde::io::ext_repr::{ExtContainer, ExtItem, ExtShape};
use ironnest_cde::io::import::Importer;

/// The `lbf` reference configuration (config.rs `LBFConfig::default`).
fn cde_config() -> CDEConfig {
    CDEConfig {
        quadtree_depth: 5,
        cd_threshold: 16,
        item_surrogate_config: SPSurrogateConfig {
            n_pole_limits: [(100, 0.0), (20, 0.75), (10, 0.90)],
            n_ff_poles: 2,
            n_ff_piers: 0,
        },
    }
}

fn rect(x_min: f64, y_min: f64, width: f64, height: f64) -> ExtShape {
    ExtShape::Rectangle {
        x_min,
        y_min,
        width,
        height,
    }
}

/// A 10×10 container with a single 2×2 item, no rotation, no min-separation.
fn importer_container_item() -> (
    Importer,
    ironnest_cde::entities::Container,
    ironnest_cde::entities::Item,
) {
    let importer = Importer::new(cde_config(), None, None, None);
    let ext_container = ExtContainer {
        id: 0,
        shape: rect(0.0, 0.0, 10.0, 10.0),
        zones: vec![],
    };
    let ext_item = ExtItem {
        id: 0,
        allowed_orientations: Some(vec![0.0]),
        shape: rect(0.0, 0.0, 2.0, 2.0),
        min_quality: None,
    };
    let container = importer
        .import_container(&ext_container)
        .expect("import container");
    let item = importer.import_item(&ext_item).expect("import item");
    (importer, container, item)
}

#[test]
fn item_fully_inside_container_is_feasible() {
    let (_importer, container, item) = importer_container_item();
    let mut layout = Layout::new(container);
    // Items are centroid-centered at import, so translating to (5,5) centers the 2×2 item in the
    // 10×10 container → fully inside → no collision with the exterior hazard.
    layout.place_item(&item, DTransformation::new(0.0, (5.0, 5.0)));
    assert!(layout.is_feasible(), "centered item should be feasible");
}

#[test]
fn item_crossing_boundary_is_infeasible() {
    let (_importer, container, item) = importer_container_item();
    let mut layout = Layout::new(container);
    // Centroid at (0.5, 0.5): the 2×2 item spans (-0.5,-0.5)..(1.5,1.5), crossing the container
    // boundary → collides with the exterior → infeasible. Exercises the CDE after the f64 flip.
    layout.place_item(&item, DTransformation::new(0.0, (0.5, 0.5)));
    assert!(
        !layout.is_feasible(),
        "item crossing the boundary must be infeasible"
    );
}

#[test]
fn place_remove_restores_feasibility() {
    let (_importer, container, item) = importer_container_item();
    let mut layout = Layout::new(container);
    let a = layout.place_item(&item, DTransformation::new(0.0, (3.0, 3.0)));
    let b = layout.place_item(&item, DTransformation::new(0.0, (3.5, 3.5)));
    assert!(!layout.is_feasible(), "two overlapping items collide");
    layout.remove_item(b);
    assert!(
        layout.is_feasible(),
        "removing the overlapping item restores feasibility"
    );
    let _ = a;
}

#[test]
fn save_restore_is_deterministic() {
    let (_importer, container, item) = importer_container_item();
    let mut layout = Layout::new(container);
    layout.place_item(&item, DTransformation::new(0.0, (5.0, 5.0)));
    let snap = layout.save();
    // Mutate, then restore: feasibility must return to the saved (feasible) state deterministically.
    layout.place_item(&item, DTransformation::new(0.0, (5.5, 5.5)));
    assert!(!layout.is_feasible());
    layout.restore(&snap);
    assert!(
        layout.is_feasible(),
        "restore returns to the saved feasible state"
    );
}
