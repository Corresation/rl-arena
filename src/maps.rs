use std::sync::Once;

const SOCCAR_DATA: &[u8] = include_bytes!("maps/soccar.bin");
const SOCCAR_MESH_SPANS: [(usize, usize); 16] = [
    (0, 16_364),
    (16_364, 16_364),
    (32_728, 16_364),
    (49_092, 16_364),
    (65_456, 18_236),
    (83_692, 18_236),
    (101_928, 18_236),
    (120_164, 18_236),
    (138_400, 416),
    (138_816, 416),
    (139_232, 2_480),
    (141_712, 2_480),
    (144_192, 2_480),
    (146_672, 2_480),
    (149_152, 416),
    (149_568, 416),
];
static INIT: Once = Once::new();

pub(crate) fn init() {
    INIT.call_once(|| {
        let meshes = soccar_meshes();
        rocketsim_rs::init_from_mem(&meshes, &[]);
    });
}

fn soccar_meshes() -> [&'static [u8]; SOCCAR_MESH_SPANS.len()] {
    SOCCAR_MESH_SPANS.map(|(offset, size)| &SOCCAR_DATA[offset..offset + size])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soccar_meshes_have_valid_cmf_layout() {
        let meshes = soccar_meshes();
        let mut expected_offset = 0;

        for (&(offset, size), mesh) in SOCCAR_MESH_SPANS.iter().zip(meshes) {
            assert_eq!(offset, expected_offset);
            assert_eq!(mesh.len(), size);

            let triangle_count = u32::from_le_bytes(mesh[0..4].try_into().unwrap()) as usize;
            let vertex_count = u32::from_le_bytes(mesh[4..8].try_into().unwrap()) as usize;
            let vertex_data_offset = 8 + triangle_count * 12;

            assert_eq!(mesh.len(), vertex_data_offset + vertex_count * 12);

            for index in mesh[8..vertex_data_offset].chunks_exact(4) {
                let index = u32::from_le_bytes(index.try_into().unwrap()) as usize;
                assert!(index < vertex_count);
            }

            expected_offset += size;
        }

        assert_eq!(expected_offset, SOCCAR_DATA.len());
    }
}
