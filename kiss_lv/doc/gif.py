from PIL import Image
import os

def compress_gif(input_folder, output_folder):
    # 确保输出文件夹存在
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # 遍历文件夹中的所有GIF文件
    for filename in os.listdir(input_folder):
        if filename.endswith('.gif'):
            # 输入文件路径
            input_path = os.path.join(input_folder, filename)
            # 输出文件路径
            output_path = os.path.join(output_folder, filename)
            
            # 打开GIF文件
            with Image.open(input_path) as img:
                # 获取原始图片尺寸
                width, height = img.size
                # 压缩为原来的一半
                new_width = width // 2
                new_height = height // 2

                # 创建一个空的列表，保存每一帧的压缩图像
                frames = []
                durations = []  # 用于保存每一帧的持续时间
                for frame in range(img.n_frames):
                    img.seek(frame)  # 切换到第 `frame` 帧
                    frame_img = img.copy()
                    frame_img = frame_img.resize((new_width, new_height))  # 压缩每一帧的图像
                    frames.append(frame_img)
                    durations.append(img.info['duration'])  # 获取每帧的持续时间

                # 保存压缩后的GIF文件，保持每一帧的持续时间
                frames[0].save(output_path, save_all=True, append_images=frames[1:], loop=0, duration=durations)
                print(f'Compressed {filename} and saved to {output_path}')

if __name__ == "__main__":
    input_folder = "./gif"  # 替换为你的输入文件夹路径
    output_folder = "./new"  # 替换为你的输出文件夹路径

    compress_gif(input_folder, output_folder)

