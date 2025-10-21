import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import torchvision.transforms as transforms
from PIL import Image
import os # 用于文件和路径操作
from sklearn.model_selection import train_test_split # 用于划分训练集和验证集
import numpy as np # 用于数值操作

# 定义模型类
class DigitRecognizer(nn.Module):
    """
    一个简单的用于数字识别的卷积神经网络 (CNN)。
    假设输入图像是 20x28 像素的灰度图。
    """
    def __init__(self, num_classes=9): # num_classes 为 9 (数字 1-5, 特定图形 6outpost, 7guard, 8base, 负样本 9neg)
        super(DigitRecognizer, self).__init__()

        # --- 卷积层 ---
        # 输入通道 1 (灰度图), 输出通道 32, 卷积核 3x3, 步长 1, 填充 1
        # 输出尺寸: (20 - 3 + 2*1)/1 + 1 = 20, (28 - 3 + 2*1)/1 + 1 = 28 -> (32, 20, 28)
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        # 输入通道 32, 输出通道 64
        # 输出尺寸: (20, 28) -> (64, 20, 28)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)

        # --- 池化层 ---
        # Max Pooling: 尺寸减半 (20x28 -> 10x14)
        # 输出尺寸: (64, 10, 14)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2, padding=0)

        # --- Dropout 层 ---
        self.dropout1 = nn.Dropout(0.25) # 卷积后
        self.dropout2 = nn.Dropout(0.5)  # 全连接前

        # --- 全连接层 ---
        # 经过卷积和池化后，特征图尺寸为 (64, 10, 14)
        # 展平后的大小是 64 * 10 * 14 = 8960
        self.fc1 = nn.Linear(64 * 10 * 14, 128) # 输入维度修正
        self.fc2 = nn.Linear(128, 64)
        self.fc3 = nn.Linear(64, num_classes) # 输出类别数

        # --- 激活函数 ---
        self.relu = nn.ReLU()
        # <0==0, >0==x

    def forward(self, x):
        """
        定义模型的前向传播过程。
        :param x: 输入张量，形状为 (batch_size, 1, 20, 28)
        :return: 模型输出张量，形状为 (batch_size, num_classes)
        """
        # --- 卷积和池化块 ---
        x = self.conv1(x)      # (batch, 1, 20, 28) -> (batch, 32, 20, 28)
        x = self.relu(x)

        x = self.conv2(x)      # (batch, 32, 20, 28) -> (batch, 64, 20, 28)
        x = self.relu(x)

        x = self.pool(x)       # (batch, 64, 20, 28) -> (batch, 64, 10, 14)
        x = self.dropout1(x)

        # --- 展平 ---
        x = x.view(x.size(0), -1) # (batch, 64*10*14) -> (batch, 8960)

        # --- 全连接块 ---
        x = self.fc1(x)        # (batch, 8960) -> (batch, 128)
        x = self.relu(x)
        x = self.dropout2(x)
        x = self.fc2(x)        # (batch, 128) -> (batch, 64)
        x = self.relu(x)
        x = self.fc3(x)        # (batch, 64) -> (batch, 9)

        return x


# --- 自定义数据集类 ---
class ImageDataset(Dataset):
    """
    用于加载图像数据的自定义数据集类。
    """
    def __init__(self, root_dir, transform=None):
        """
        :param root_dir: 数据集根目录 (例如 'data_set'，相对于 test.py 的位置)
        :param transform: 可选的图像预处理变换
        """
        # 修正：使用传入的 root_dir 参数，而不是硬编码
        self.root_dir = root_dir
        self.transform = transform
        self.images = []
        self.labels = []

        # 定义类别名称到数字标签的映射
        # 假设 '1' -> 0, '2' -> 1, ..., '9neg' -> 8
        # 请根据你的实际文件夹名调整键名！
        # 例如，如果文件夹是 '6outpost'，则键应为 '6outpost'
        # 如果是 '7guard'，则键应为 '7guard'
        self.class_to_idx = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, 
                             '6outpost': 5, '7guard': 6, '8base': 7, '9neg': 8}

        # 遍历每个类别文件夹
        for class_name in sorted(os.listdir(root_dir)): # 遍历根目录下的文件夹
            class_path = os.path.join(root_dir, class_name)
            if os.path.isdir(class_path): # 确保是文件夹
                if class_name not in self.class_to_idx:
                    print(f"Warning: Found directory '{class_name}' not in class mapping. Skipping.")
                    continue # 跳过不在映射中的文件夹

                label = self.class_to_idx[class_name]
                # 遍历该类别文件夹下的所有图像文件
                for img_name in os.listdir(class_path):
                    if img_name.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
                        img_path = os.path.join(class_path, img_name)
                        self.images.append(img_path)
                        self.labels.append(label)

    def __len__(self):
        return len(self.images)

    def __getitem__(self, idx):
        img_path = self.images[idx]
        label = self.labels[idx]

        # 加载图像
        image = Image.open(img_path).convert('L') # 转换为灰度图

        # 应用预处理变换
        if self.transform:
            image = self.transform(image)
        else:
            # 如果没有提供 transform，手动转换为张量并归一化
            image = transforms.ToTensor()(image) # ToTensor 会自动归一化到 [0, 1]

        # 注意：Image.open 加载的图像是 (H, W)，而 PyTorch CNN 期望 (C, H, W)
        # ToTensor 已经处理了通道维度，所以 image 现在是 (1, 20, 28) 或 (20, 28) -> ToTensor -> (1, 20, 28)

        return image, label


# --- 训练函数 ---
def train_model(model, train_loader, criterion, optimizer, num_epochs, device):
    """
    训练模型的函数。
    注意：criterion 和 optimizer 作为参数传入，不应在此函数内部重新定义。
    """
    model.train() # 设置模型为训练模式
    for epoch in range(num_epochs):
        running_loss = 0.0
        correct = 0
        total = 0
        for batch_idx, (data, target) in enumerate(train_loader):
            # 将数据和标签移动到指定设备 (CPU 或 GPU)
            data, target = data.to(device), target.to(device)

            # 1. 清零梯度
            optimizer.zero_grad()

            # 2. 前向传播,预测输出
            output = model(data)

            # 3. 计算损失
            loss = criterion(output, target)

            # 4. 反向传播：计算梯度
            loss.backward()

            # 5. 更新参数
            optimizer.step()

            running_loss += loss.item()
            _, predicted = output.max(1) # 获取最大值对应的索引
            total += target.size(0)
            correct += predicted.eq(target).sum().item() # 计算正确预测的数量

            if batch_idx % 100 == 0: # 每处理 100 个批次打印一次信息
                print(f'Epoch: {epoch+1}, Batch: {batch_idx}, Loss: {loss.item():.6f}')

        epoch_loss = running_loss / len(train_loader)
        epoch_acc = 100. * correct / total
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss:.4f}, Acc: {epoch_acc:.2f}%')

# --- 验证函数 ---
def validate_model(model, val_loader, criterion, device):
    """
    验证模型的函数。
    """
    model.eval() # 设置模型为评估模式
    val_loss = 0
    correct = 0
    total = 0

    with torch.no_grad(): # 在验证时禁用梯度计算，节省内存和计算
        for data, target in val_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            val_loss += criterion(output, target).item()  # 累加损失
            _, predicted = output.max(1)
            total += target.size(0)
            correct += predicted.eq(target).sum().item()

    val_loss /= len(val_loader) # 计算平均损失
    val_acc = 100. * correct / total
    print(f'Validation set: Average loss: {val_loss:.4f}, Accuracy: {val_acc:.2f}%\n')
    return val_acc # 返回验证准确率，可用于早停等策略

# --- 主程序 ---
if __name__ == "__main__":
    # --- 1. 设置设备 ---
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # --- 2. 准备数据 ---
    # 定义数据集根目录
    # 确保 'data_set' 文件夹与 'test.py' 在同一目录下
    data_root = "data_set" # 相对于 test.py 的路径

    # 定义图像预处理变换
    # Resize: 确保所有图像都是 20x28 (虽然你的图像是这个尺寸，但写上更安全)
    # ToTensor: 将 PIL Image 或 numpy.ndarray 转换为 FloatTensor，并将像素值从 [0, 255] 归一化到 [0, 1]
    transform = transforms.Compose([
        transforms.Resize((20, 28)), # 确保尺寸
        transforms.ToTensor(),      # 转换为张量并归一化
        # transforms.Normalize((0.5,), (0.5,)) # 可选：进一步归一化到 [-1, 1]
    ])

    # 创建数据集实例
    full_dataset = ImageDataset(root_dir=data_root, transform=transform)

    # 检查数据集是否加载成功
    if len(full_dataset) == 0:
        print("Error: No images found in the dataset. Please check the directory structure and file extensions.")
        exit()

    print(f"Total images loaded: {len(full_dataset)}")
    print(f"Class mapping: {full_dataset.class_to_idx}")

    # --- 3. 划分训练集和验证集 ---
    # 通常将数据划分为训练集和验证集，以监控模型在未见过的数据上的表现
    # 这里使用 sklearn 的 train_test_split
    indices = list(range(len(full_dataset)))
    train_indices, val_indices = train_test_split(indices, test_size=0.2, random_state=42, stratify=full_dataset.labels) # stratify 保证各类别比例一致

    train_dataset = torch.utils.data.Subset(full_dataset, train_indices)
    val_dataset = torch.utils.data.Subset(full_dataset, val_indices)

    # 创建数据加载器 (DataLoader)
    batch_size = 32 # 可以根据你的内存调整
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False) # 验证集通常不打乱

    # --- 4. 创建模型、损失函数和优化器 ---
    # 修正：num_classes 为 9 (1, 2, 3, 4, 5, 6outpost, 7guard, 8base, 9neg)
    num_classes = 9
    model = DigitRecognizer(num_classes=num_classes).to(device)

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # --- 5. 训练模型 ---
    num_epochs = 10 # 训练轮数，你可以根据需要调整
    print("Starting Training...")
    best_val_acc = 0.0
    for epoch in range(num_epochs):
        print(f"--- Epoch {epoch+1}/{num_epochs} ---")
        train_model(model, train_loader, criterion, optimizer, 1, device) # 每次只训练一个 epoch
        val_acc = validate_model(model, val_loader, criterion, device)

        # 简单的模型保存逻辑：保存验证集上表现最好的模型
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), "best_digit_recognizer_model.pth")
            print(f"New best model saved with val acc: {best_val_acc:.2f}%")

    print("Training finished.")
    print(f"Best validation accuracy achieved: {best_val_acc:.2f}%")

    # --- 6. 加载最佳模型进行最终评估 (可选) ---
    # model.load_state_dict(torch.load("best_digit_recognizer_model.pth"))
    # validate_model(model, val_loader, criterion, device) # 在验证集上评估最终模型