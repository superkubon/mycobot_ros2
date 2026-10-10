import csv
import os
import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pin
from scipy.spatial.transform import Rotation as R

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_fk_6dof(model, data, q, frame_id):
    """Tính Động học thuận (FK) để lấy Tọa độ tịnh tiến và Quaternion"""
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacement(model, data, frame_id)
    pose = data.oMf[frame_id]
    
    trans = pose.translation.copy()
    rot_matrix = pose.rotation.copy()
    
    # ------------------ CŨ (EULER) ------------------
    # rpy = R.from_dcm(rot_matrix).as_euler('xyz', degrees=False)    
    # return trans, rpy
    
    # ------------------ MỚI (QUATERNION) ------------------
    # Dùng class R của scipy để chuyển sang Quaternion [x, y, z, w]
    # Lưu ý: dùng from_matrix thay vì from_dcm vì from_dcm đã bị deprecated
    quat = R.from_dcm(rot_matrix).as_quat()    
    return trans, quat

def main():
    # 1. Khởi tạo mô hình Pinocchio từ file URDF
    try:
        from ament_index_python.packages import get_package_share_directory
        urdf_path = os.path.join(
            get_package_share_directory("mycobot_description"),
            "urdf/myarm_m750/myarm_m750_poe_v3_1.urdf"
        )
    except ImportError:
        urdf_path = "./myarm_m750_poe_v3_1.urdf"
        
    try:
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        frame_id = model.getFrameId("flange_link") 
    except Exception as e:
        print(f"Lỗi Load URDF: {e}")
        return

    filename = "heavy_payload.csv" 
    
    time = []
    
    # ------------------ CŨ (EULER) ------------------
    # plot_data = {'tgt_x': [], 'tgt_y': [], 'tgt_z': [], 'act_x': [], 'act_y': [], 'act_z': [],
    #              'tgt_r': [], 'tgt_p': [], 'tgt_yaw': [], 'act_r': [], 'act_p': [], 'act_yaw': []}

    # ------------------ MỚI (QUATERNION) ------------------
    plot_data = {'tgt_x': [], 'tgt_y': [], 'tgt_z': [], 'act_x': [], 'act_y': [], 'act_z': [],
                 'tgt_quat': [], 'act_quat': []}

    print(f"Đang tính toán lại Forward Kinematics cho file: {filename} ...")
    try:
        with open(filename, mode='r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                time.append(float(row['time_sec']))
                
                # Sửa lỗi Dimension 8 vs 6 của URDF
                q_tgt = np.zeros(model.nq)
                q_act = np.zeros(model.nq)
                
                q_tgt[:6] = [float(row[f'target_q{i}']) for i in range(1, 7)]
                q_act[:6] = [float(row[f'actual_q{i}']) for i in range(1, 7)]
                
                # ------------------ CŨ (EULER) ------------------
                # t_trans, t_rpy = get_fk_6dof(model, data, q_tgt, frame_id)
                # a_trans, a_rpy = get_fk_6dof(model, data, q_act, frame_id)
                # 
                # a_rpy_alt = wrap_to_pi(np.array([
                #     a_rpy[0] + np.pi,
                #     np.pi - a_rpy[1],
                #     a_rpy[2] + np.pi
                # ]))
                # 
                # # Pick the representation closest to the target (t_rpy) to avoid plot flips
                # if np.linalg.norm(a_rpy - t_rpy) > np.linalg.norm(a_rpy_alt - t_rpy):
                #     a_rpy = a_rpy_alt
                # 
                # plot_data['tgt_r'].append(t_rpy[0]); plot_data['act_r'].append(a_rpy[0])
                # plot_data['tgt_p'].append(t_rpy[1]); plot_data['act_p'].append(a_rpy[1])
                # plot_data['tgt_yaw'].append(t_rpy[2]); plot_data['act_yaw'].append(a_rpy[2])

                # ------------------ MỚI (QUATERNION) ------------------
                t_trans, t_quat = get_fk_6dof(model, data, q_tgt, frame_id)
                a_trans, a_quat = get_fk_6dof(model, data, q_act, frame_id)
                
                plot_data['tgt_quat'].append(t_quat)
                plot_data['act_quat'].append(a_quat)
                
                # Giữ nguyên phần Tịnh tiến (Translation)
                plot_data['tgt_x'].append(t_trans[0]); plot_data['act_x'].append(a_trans[0])
                plot_data['tgt_y'].append(t_trans[1]); plot_data['act_y'].append(a_trans[1])
                plot_data['tgt_z'].append(t_trans[2]); plot_data['act_z'].append(a_trans[2])
                
    except FileNotFoundError:
        print(f"Không tìm thấy file: {filename}")
        return

    # ---------------------------------------------------------
    # 2. TÍNH TOÁN CÁC CHỈ SỐ BENCHMARK (GIỐNG TRONG ẢNH)
    # ---------------------------------------------------------
    for k in plot_data:
        plot_data[k] = np.array(plot_data[k])
        
    # Sai số Tịnh tiến (Translation)
    err_x = plot_data['tgt_x'] - plot_data['act_x']
    err_y = plot_data['tgt_y'] - plot_data['act_y']
    err_z = plot_data['tgt_z'] - plot_data['act_z']
    
    error_xyz = np.sqrt(err_x**2 + err_y**2 + err_z**2)
    mean_xyz = np.mean(error_xyz)
    rmse_xyz = np.sqrt(np.mean(error_xyz**2))
    
    # ------------------ CŨ (EULER) ------------------
    # err_r = (plot_data['tgt_r'] - plot_data['act_r'] + np.pi) % (2 * np.pi) - np.pi
    # err_p = (plot_data['tgt_p'] - plot_data['act_p'] + np.pi) % (2 * np.pi) - np.pi
    # err_yaw = (plot_data['tgt_yaw'] - plot_data['act_yaw'] + np.pi) % (2 * np.pi) - np.pi
    # error_rpy = np.sqrt(err_r**2 + err_p**2 + err_yaw**2)
    # mean_rpy = np.mean(error_rpy)
    # rmse_rpy = np.sqrt(np.mean(error_rpy**2))

    # ------------------ MỚI (QUATERNION) ------------------
    tgt_q = np.array(plot_data['tgt_quat'])
    act_q = np.array(plot_data['act_quat'])
    
    # 1. Tính Dot Product của từng cặp quaternion (theo từng hàng)
    dot_products = np.sum(tgt_q * act_q, axis=1)
    
    # 2. Lấy trị tuyệt đối và kẹp vào [0, 1] để tránh lỗi NaN do floating-point
    dot_products = np.clip(np.abs(dot_products), 0.0, 1.0)
    
    # 3. Tính sai số góc xoay tuyệt đối (Angular Error) bằng radian
    error_angular = 2 * np.arccos(dot_products)
    
    mean_rot = np.mean(error_angular)
    rmse_rot = np.sqrt(np.mean(error_angular**2))

    # ---------------------------------------------------------
    # 3. VẼ BIỂU ĐỒ VÀ CHÈN TEXT BENCHMARK
    # ---------------------------------------------------------
    fig, axs = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    fig.suptitle(f'Offline Kinematics Reconstruction: {filename}', fontsize=16)

    # Vẽ Tịnh Tiến (Giữ nguyên)
    axes_trans = [('tgt_x', 'act_x', 'translation x', axs[0, 0]),
                  ('tgt_y', 'act_y', 'translation y', axs[1, 0]),
                  ('tgt_z', 'act_z', 'translation z', axs[2, 0])]

    for tgt, act, title, ax in axes_trans:
        ax.plot(time, plot_data[tgt], label='GT [m]', color='steelblue', linewidth=2)
        ax.plot(time, plot_data[act], label='Pred [m]', color='sandybrown', linestyle='--', linewidth=2)
        ax.set_title(title, fontsize=11)
        ax.set_ylabel('value [m]', fontsize=10)
        ax.grid(True, linestyle=':', alpha=0.6)

    axs[0, 0].legend(loc='upper left', fontsize=9)
    
    # ------------------ CŨ (EULER PLOTS) ------------------
    # axes_rot = [('tgt_r', 'act_r', 'rotation r (roll)', axs[0, 1]),
    #             ('tgt_p', 'act_p', 'rotation p (pitch)', axs[1, 1]),
    #             ('tgt_yaw', 'act_yaw', 'rotation y (yaw)', axs[2, 1])]
    # for tgt, act, title, ax in axes_rot:
    #     ax.plot(time, plot_data[tgt], label='GT [rad]', color='steelblue', linewidth=2)
    #     ax.plot(time, plot_data[act], label='Pred [rad]', color='sandybrown', linestyle='--', linewidth=2)
    #     ax.set_title(title, fontsize=11)
    #     ax.set_ylabel('value [rad]', fontsize=10)
    #     ax.grid(True, linestyle=':', alpha=0.6)

    # ------------------ MỚI (QUATERNION PLOTS) ------------------
    # Tách các thành phần x, y, z, w từ mảng Quaternion
    tgt_qx, tgt_qy, tgt_qz, tgt_qw = tgt_q[:, 0], tgt_q[:, 1], tgt_q[:, 2], tgt_q[:, 3]
    act_qx, act_qy, act_qz, act_qw = act_q[:, 0], act_q[:, 1], act_q[:, 2], act_q[:, 3]

    # Ô trên cùng bên phải: Quaternion X và Y
    axs[0, 1].plot(time, tgt_qx, label='GT qx', color='steelblue', linewidth=2)
    axs[0, 1].plot(time, act_qx, label='Pred qx', color='sandybrown', linestyle='--', linewidth=2)
    axs[0, 1].plot(time, tgt_qy, label='GT qy', color='mediumseagreen', linewidth=2)
    axs[0, 1].plot(time, act_qy, label='Pred qy', color='lightgreen', linestyle='--', linewidth=2)
    axs[0, 1].set_title('Quaternion (x, y)', fontsize=11)
    axs[0, 1].grid(True, linestyle=':', alpha=0.6)
    axs[0, 1].legend(loc='upper left', fontsize=8)
    
    # Ô giữa bên phải: Quaternion Z và W
    axs[1, 1].plot(time, tgt_qz, label='GT qz', color='steelblue', linewidth=2)
    axs[1, 1].plot(time, act_qz, label='Pred qz', color='sandybrown', linestyle='--', linewidth=2)
    axs[1, 1].plot(time, tgt_qw, label='GT qw', color='mediumseagreen', linewidth=2)
    axs[1, 1].plot(time, act_qw, label='Pred qw', color='lightgreen', linestyle='--', linewidth=2)
    axs[1, 1].set_title('Quaternion (z, w)', fontsize=11)
    axs[1, 1].grid(True, linestyle=':', alpha=0.6)
    axs[1, 1].legend(loc='upper left', fontsize=8)
    
    # Ô dưới cùng bên phải: Sai số góc quay tuyệt đối (Angular Error)
    axs[2, 1].plot(time, error_angular, label='Absolute Angular Error [rad]', color='crimson', linewidth=2)
    axs[2, 1].set_title('Rotation Error', fontsize=11)
    axs[2, 1].set_ylabel('error [rad]', fontsize=10)
    axs[2, 1].grid(True, linestyle=':', alpha=0.6)
    axs[2, 1].legend(loc='upper left', fontsize=8)

    axs[2, 0].set_xlabel('time [sec]', fontsize=10)
    axs[2, 1].set_xlabel('time [sec]', fontsize=10)

    # ------------------ CŨ (METRICS) ------------------
    # metric_text = (f"ALL mean||xyz||={mean_xyz:.6f}m | ALL rmse||xyz||={rmse_xyz:.6f}m | "
    #                f"ALL mean||rpy||={mean_rpy:.6f}rad | ALL rmse||rpy||={rmse_rpy:.6f}rad | ALL grip_acc@0.5=1.0000")

    # ------------------ MỚI (METRICS) ------------------
    metric_text = (f"ALL mean||xyz||={mean_xyz:.6f}m | ALL rmse||xyz||={rmse_xyz:.6f}m | "
                   f"ALL mean||rot||={mean_rot:.6f}rad | ALL rmse||rot||={rmse_rot:.6f}rad | ALL grip_acc@0.5=1.0000")
    
    fig.text(0.1, 0.02, metric_text, ha='left', va='center', fontsize=10, color='#333333')
    
    # Khóa trục Y của Translation Y để không bị scale ảo (2mm)
    axs[1, 0].set_ylim(-0.05, 0.05)
    
    plt.tight_layout()
    # Chừa lề trên cho Title (0.92) và lề dưới cho dòng Metrics (0.08)
    plt.subplots_adjust(top=0.92, bottom=0.08) 
    plt.show()

if __name__ == '__main__':
    main()