import os
import shutil
import numpy as np
from glob import glob
import cv2

def convert_sun3d_to_deltas():
    src_base = "/root/Final_SLAM/mit_w85k1/whole_apartment"
    dst_base = "/root/Final_SLAM/DELTAS/assets/SUN3D_Data/scannet_sample"
    
    # 创建目录
    os.makedirs(dst_base, exist_ok=True)
    scans_test_dir = os.path.join(dst_base, "scans_test")
    os.makedirs(scans_test_dir, exist_ok=True)
    
    # 读取数据集文件
    rgb_paths = sorted(glob(os.path.join(src_base, "image", "*")))[:10642]
    depth_paths = sorted(glob(os.path.join(src_base, "depth", "*")))[:10642]
    intrinsics_path = os.path.join(src_base, "intrinsics.txt")
    extrinsics_path = os.path.join(src_base, "extrinsics", "20130512130736.txt")
    
    print(f"找到 {len(rgb_paths)} 张RGB图像和 {len(depth_paths)} 张深度图")
    
    # 读取内参矩阵
    intrinsics_3x3 = np.loadtxt(intrinsics_path)
    print(f"读取内参矩阵: {intrinsics_3x3.shape}")
    
    # 将3×3内参矩阵转换为4×4矩阵（添加齐次坐标）
    intrinsics_4x4 = np.eye(4)
    intrinsics_4x4[:3, :3] = intrinsics_3x3
    
    # 提取内参参数
    fx = intrinsics_3x3[0, 0]
    fy = intrinsics_3x3[1, 1]
    cx = intrinsics_3x3[0, 2]
    cy = intrinsics_3x3[1, 2]
    
    # 读取外参矩阵
    extrinsics_data = np.loadtxt(extrinsics_path)
    num_poses = len(rgb_paths)
    extrinsics_list = []
    
    # 处理外参矩阵
    for i in range(num_poses):
        mat_3x4 = extrinsics_data[i*3:(i+1)*3, :]
        mat_4x4 = np.vstack([mat_3x4, [0, 0, 0, 1]])
        extrinsics_list.append(mat_4x4)
    
    print(f"处理了 {len(extrinsics_list)} 个相机位姿")
    
    # 设置参数
    seq_length = 10642
    seq_gap = 1
    
    # 创建场景列表文件
    scenes_list_path = os.path.join("/root/Final_SLAM/DELTAS/assets/SUN3D_Data", "scannetv2_test.txt")
    scenes_list = []
    
    # 组织数据
    scene_count = 0
    for start_idx in range(0, len(rgb_paths) - (seq_length-1) * seq_gap, seq_length * seq_gap):
        if start_idx + (seq_length-1) * seq_gap >= len(rgb_paths):
            break
        
        scene_id = f"scene{scene_count:04d}_00"
        scene_dir = os.path.join(scans_test_dir, scene_id)
        
        # 创建子目录
        os.makedirs(os.path.join(scene_dir, "color"), exist_ok=True)
        os.makedirs(os.path.join(scene_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(scene_dir, "pose"), exist_ok=True)
        os.makedirs(os.path.join(scene_dir, "intrinsic"), exist_ok=True)
        
        # 添加到场景列表
        scenes_list.append(scene_id)
        
        # 创建场景配置文件
        scene_config_path = os.path.join(scene_dir, f"{scene_id}.txt")
        with open(scene_config_path, 'w') as f:
            f.write(f"colorHeight = 240\n")
            f.write(f"colorToDepthExtrinsics = 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n")
            f.write(f"colorWidth = 320\n")
            f.write(f"depthHeight = 240\n")
            f.write(f"depthWidth = 320\n")
            f.write(f"fx_color = {fx}\n")
            f.write(f"fx_depth = {fx}\n")
            f.write(f"fy_color = {fy}\n")
            f.write(f"fy_depth = {fy}\n")
            f.write(f"mx_color = {cx}\n")
            f.write(f"mx_depth = {cx}\n")
            f.write(f"my_color = {cy}\n")
            f.write(f"my_depth = {cy}\n")
            f.write(f"numColorFrames = {seq_length}\n")
            f.write(f"numDepthFrames = {seq_length}\n")
        
        # 复制并处理每一帧
        for i in range(seq_length):
            idx = start_idx + i * seq_gap
            
            # 处理RGB图像和深度图
            rgb_src = rgb_paths[idx]
            rgb_dst = os.path.join(scene_dir, "color", f"{i}.jpg")
            
            img = cv2.imread(rgb_src)
            img = cv2.resize(img, (320, 240))
            cv2.imwrite(rgb_dst, img)
            
            depth_src = depth_paths[idx]
            depth_dst = os.path.join(scene_dir, "depth", f"{i}.png")
            
            depth = cv2.imread(depth_src, cv2.IMREAD_UNCHANGED)
            depth = cv2.resize(depth, (320, 240), interpolation=cv2.INTER_NEAREST)
            cv2.imwrite(depth_dst, depth)
            
            # 保存位姿
            pose = extrinsics_list[idx]
            pose_path = os.path.join(scene_dir, "pose", f"{i}.txt")
            np.savetxt(pose_path, pose, fmt='%.6f')
        
        # 创建intrinsic和extrinsic文件
        intrinsic_dir = os.path.join(scene_dir, "intrinsic")
        
        # 1. intrinsic_color.txt - 彩色相机内参（4×4矩阵）
        intrinsic_color_path = os.path.join(intrinsic_dir, "intrinsic_color.txt")
        np.savetxt(intrinsic_color_path, intrinsics_4x4, fmt='%.6f')
        
        # 2. intrinsic_depth.txt - 深度相机内参（4×4矩阵，假设与彩色相机相同）
        intrinsic_depth_path = os.path.join(intrinsic_dir, "intrinsic_depth.txt")
        np.savetxt(intrinsic_depth_path, intrinsics_4x4, fmt='%.6f')
        
        # 3. extrinsic_color.txt - 彩色相机外参（使用单位矩阵）
        extrinsic_color_path = os.path.join(intrinsic_dir, "extrinsic_color.txt")
        identity_matrix = np.eye(4)
        np.savetxt(extrinsic_color_path, identity_matrix, fmt='%.6f')
        
        # 4. extrinsic_depth.txt - 深度相机外参（使用单位矩阵）
        extrinsic_depth_path = os.path.join(intrinsic_dir, "extrinsic_depth.txt")
        np.savetxt(extrinsic_depth_path, identity_matrix, fmt='%.6f')
        
        scene_count += 1
        if scene_count % 10 == 0:
            print(f"已处理 {scene_count} 个场景")
    
    # 保存场景列表
    with open(scenes_list_path, 'w') as f:
        for scene in scenes_list:
            f.write(f"{scene}\n")
    
    print(f"转换完成！共创建了 {scene_count} 个场景，保存在 {dst_base}")
    print(f"场景列表已保存到 {scenes_list_path}")

if __name__ == "__main__":
    convert_sun3d_to_deltas()