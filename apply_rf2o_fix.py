#!/usr/bin/env python3
"""Correctifs ISIMM : RF2O robuste (NaN) + nodes Python + SLAM.

Usage (sur le Pi) :  python3 apply_rf2o_fix.py [~/isimm]
- sauvegarde chaque fichier modifié en *.bak
- tout-ou-rien par fichier : si un point d'ancrage ne correspond pas, le fichier n'est PAS touché
- idempotent (marqueur ISIMM-FIX)
"""
import os
import re
import shutil
import sys

root = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/isimm')
src = os.path.join(root, 'ros2_ws', 'src')
pkg = os.path.join(src, 'isimm_robot')
MARK_CPP = '// ISIMM-FIX\n'
MARK_PY = '# ISIMM-FIX\n'
MARK_YAML = '# ISIMM-FIX\n'
all_ok = True


def patch(path, edits, marker):
    """edits = [(nom, regex, remplacement(str|callable), nb_attendu)]."""
    global all_ok
    if not os.path.isfile(path):
        print(f'[ABSENT ] {path}')
        all_ok = False
        return
    s = open(path, encoding='utf-8').read()
    if marker in s:
        print(f'[DEJA   ] {os.path.relpath(path, root)}')
        return
    for name, pat, rep, want in edits:
        fn = rep if callable(rep) else (lambda m, r=rep: r)
        s, n = re.subn(pat, fn, s, flags=re.S)
        if n != want:
            print(f'[ECHEC  ] {os.path.relpath(path, root)} : "{name}" '
                  f'{n} occurrence(s), {want} attendue(s) -> fichier NON modifié')
            all_ok = False
            return
    shutil.copy2(path, path + '.bak')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(marker + s)
    print(f'[OK     ] {os.path.relpath(path, root)} ({len(edits)} correctifs, sauvegarde .bak)')


# ------------------------------------------------------------------ RF2O (C++)
FILTER_NEW = '''if (!cov_odo.allFinite() || !kai_loc_level_.allFinite())
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[rf2o] niveau %u ignore : non fini (valid=%u, cov_finite=%d, var_finite=%d)",
      level, num_valid_range, (int)cov_odo.allFinite(), (int)kai_loc_level_.allFinite());
    transformations[level].setIdentity();
    return true;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(cov_odo);
  if (eigensolver.info() != Eigen::Success)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[rf2o] eigensolver echoue au niveau %u : niveau ignore", level);
    transformations[level].setIdentity();
    return true;
  }'''

cpp_edits = [
    ('niveaux de pyramide 5 -> 3 (scan clairsemé : ~140 points valides)',
     r'ctf_levels = 5;', 'ctf_levels = 3;', 1),
    ('auto-affectation laser_oldpose_',
     r'laser_oldpose_ = laser_oldpose_;', 'laser_oldpose_ = laser_pose_;', 1),
    ('poids : division par zéro',
     r'std::sqrt\(1\.f/w_der\)', 'std::sqrt(1.f/std::max(w_der, 1e-9f))', 1),
    ('poids : normalisation sûre',
     r'const float inv_max = 1\.f / weights\.maxCoeff\(\);',
     'const float wmax = weights.maxCoeff();\n'
     '  const float inv_max = (std::isfinite(wmax) && wmax > 0.f) ? 1.f / wmax : 0.f;', 1),
    ('k = 10/aver_dt : borne',
     r'10\.f/aver_dt;', '10.f/std::max(aver_dt, 1e-3f);', 1),
    ('régularisation AtA avant covariance',
     r'cov_odo = \(1\.f/float\(num_valid_range-3\)\)\*AtA\.inverse\(\)\*res\.squaredNorm\(\);',
     'AtA.diagonal().array() += 1e-6f;   // régularisation\n'
     '  cov_odo = (1.f/float(num_valid_range-3))*AtA.inverse()*res.squaredNorm();', 2),
    ('minimum de points valides par niveau',
     r'if \(num_valid_range > 3\)', 'if (num_valid_range > 10)', 1),
    ('filterLevelSolution : ne plus perdre la frame',
     r'Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver\(cov_odo\);\s*'
     r'if \(eigensolver\.info\(\) != Eigen::Success\)\s*\{.*?return false;\s*\}',
     FILTER_NEW, 1),
    ('garde-fou avant PoseUpdate',
     r'//Update poses\s*PoseUpdate\(\);',
     '//Update poses\n'
     '  for (auto &T : transformations)\n'
     '    if (!T.allFinite()) T.setIdentity();\n'
     '  PoseUpdate();', 1),
    ('dt nul dans le calcul des vitesses',
     r'lin_speed = acu_trans\(0,2\) / time_inc_sec;',
     'if (time_inc_sec < 1e-3) time_inc_sec = 1e-3;\n'
     '  lin_speed = acu_trans(0,2) / time_inc_sec;', 1),
]
patch(os.path.join(src, 'rf2o_laser_odometry', 'src', 'CLaserOdometry2D.cpp'),
      cpp_edits, MARK_CPP)


# ------------------------------------------------------------------ Python
def esp_except(m):
    ind = m.group(1)
    return (f'{ind}except (serial.SerialException, OSError, TypeError):'
            f'{ind}    if self._stop:{ind}        break{m.group(2)}')


patch(os.path.join(pkg, 'isimm_robot', 'esp32_sensor_node.py'), [
    ('lecture série : TypeError à la fermeture',
     r'(\n\s+)except serial\.SerialException:(\s+self\.get_logger\(\)\.error\(\'Perte du lien)',
     esp_except, 1),
    ('double shutdown',
     r'\n        rclpy\.shutdown\(\)', '\n        if rclpy.ok():\n            rclpy.shutdown()', 1),
], MARK_PY)

patch(os.path.join(pkg, 'isimm_robot', 'scan_self_filter.py'), [
    ('double shutdown',
     r'\n        rclpy\.shutdown\(\)', '\n        if rclpy.ok():\n            rclpy.shutdown()', 1),
], MARK_PY)

# ------------------------------------------------------------------ YAML
patch(os.path.join(pkg, 'config', 'slam_params.yaml'), [
    ('transform_timeout', r'transform_timeout: 0\.2', 'transform_timeout: 0.5', 1),
    ('map_update_interval (charge CPU Pi)', r'map_update_interval: 2\.0', 'map_update_interval: 5.0', 1),
], MARK_YAML)
patch(os.path.join(pkg, 'config', 'slam_localization_params.yaml'), [
    ('transform_timeout', r'transform_timeout: 0\.2', 'transform_timeout: 0.5', 1),
], MARK_YAML)
patch(os.path.join(pkg, 'config', 'robot_params.yaml'), [
    ('scan_self_filter.min_range', r'(box_half_width: 0\.20\s+margin: 0\.02\s+min_range: )0\.0',
     lambda m: m.group(1) + '0.15', 1),
], MARK_YAML)

print('\nTERMINE : ' + ('tout est appliqué.' if all_ok else 'des fichiers sont à corriger à la main (voir ci-dessus).'))
sys.exit(0 if all_ok else 1)
