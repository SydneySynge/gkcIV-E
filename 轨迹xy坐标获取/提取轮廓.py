import cv2
import numpy as np
import pandas as pd
import csv
import matplotlib.pyplot as plt

# 定义函数从图像中提取轮廓并保存到CSV文件
def extract_and_save_contours(image_path, csv_output_path):
    # 加载图像
    image = cv2.imread(image_path)

    # 转换为灰度图
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 应用Canny边缘检测
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # 寻找轮廓
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # 绘制轮廓
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
    cv2.imshow('Contours', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 存储轮廓点的坐标
    coords = []
    for cnt in contours:
        for point in cnt:
            x, y = point[0]
            coords.append([x, y])

    # 将坐标保存到CSV文件
    df = pd.DataFrame(coords)
    df.to_csv(csv_output_path, index=False)
    print(f"Contour points saved to {csv_output_path}")

# 定义函数从CSV文件加载坐标并对其进行标准化和降采样
def normalize_and_downsample(csv_input_path, downsampling_grid_size):
    # 从CSV文件加载坐标
    with open(csv_input_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        x_values, y_values = zip(*[(int(row[0]), int(row[1])) for row in csvreader])

    # 标准化坐标
    x_values_normalized = [(x - min(x_values)) / (max(x_values) - min(x_values)) * 8 - 4 for x in x_values]
    y_values_normalized = [-(y - min(y_values)) / (max(y_values) - min(y_values)) * 8 + 4 for y in y_values]

    # 降采样坐标
    downsampled_points = {}
    for x, y in zip(x_values_normalized, y_values_normalized):
        grid_x, grid_y = int(x * downsampling_grid_size), int(y * downsampling_grid_size)
        if (grid_x, grid_y) not in downsampled_points:
            downsampled_points[(grid_x, grid_y)] = (x, y)

    return list(downsampled_points.values())

# 定义函数以交互方式绘制并选择点
def interactive_plot(points):
    clicked_points = []

    def onclick(event):
        clicked_points.append((event.xdata, event.ydata))
        if len(clicked_points) > 1:
            plt.plot([p[0] for p in clicked_points[-2:]],
                     [p[1] for p in clicked_points[-2:]], 'r-')
            plt.draw()

    plt.scatter([p[0] for p in points], [p[1] for p in points])
    plt.gcf().canvas.mpl_connect('button_press_event', onclick)
    plt.show()

    return clicked_points

# 主程序执行流程
if __name__ == '__main__':
    # 提取图像中的轮廓并保存到CSV文件
    image_path = 'tsk.png'  # 路径可能需要调整
    csv_output_path = 'tsk.csv'
    extract_and_save_contours(image_path, csv_output_path)

    # 从CSV文件加载坐标，进行标准化和降采样
    downsampling_grid_size = 10  # 根据需求调整网格大小
    downsampled_points = normalize_and_downsample(csv_output_path, downsampling_grid_size)

    # 以交互方式绘制点，并允许用户选择点
    selected_points = interactive_plot(downsampled_points)

    # 打印用户选择的点
    print("Selected points:", selected_points)
