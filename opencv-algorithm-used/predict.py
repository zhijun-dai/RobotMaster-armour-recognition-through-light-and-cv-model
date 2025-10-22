
import sys
import torch
import torchvision.transforms as transforms
from PIL import Image
import numpy as np
import argparse

# 导入模型结构
class DigitRecognizer(torch.nn.Module):
    def __init__(self, num_classes=9):
        super(DigitRecognizer, self).__init__()
        self.conv1 = torch.nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)
        self.conv2 = torch.nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)
        self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2, padding=0)
        self.dropout1 = torch.nn.Dropout(0.25)
        self.dropout2 = torch.nn.Dropout(0.5)
        self.fc1 = torch.nn.Linear(64 * 10 * 14, 128)
        self.fc2 = torch.nn.Linear(128, 64)
        self.fc3 = torch.nn.Linear(64, num_classes)
        self.relu = torch.nn.ReLU()
    def forward(self, x):
        x = self.conv1(x)
        x = self.relu(x)
        x = self.conv2(x)
        x = self.relu(x)
        x = self.pool(x)
        x = self.dropout1(x)
        x = x.view(x.size(0), -1)
        x = self.fc1(x)
        x = self.relu(x)
        x = self.dropout2(x)
        x = self.fc2(x)
        x = self.relu(x)
        x = self.fc3(x)
        return x

def predict_from_stdin():
    # 20x28 单通道
    img_bytes = sys.stdin.buffer.read(20*28)
    arr = np.frombuffer(img_bytes, dtype=np.uint8).reshape((20, 28))
    img = Image.fromarray(arr, mode='L')
    return img

def predict_from_file(path):
    return Image.open(path).convert('L')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("img_path", nargs='?', default=None)
    parser.add_argument("--from-stdin", action="store_true")
    args = parser.parse_args()

    model = DigitRecognizer(num_classes=9)
    model.load_state_dict(torch.load("best_digit_recognizer_model.pth", map_location="cpu"))
    model.eval()
    transform = transforms.Compose([
        transforms.Resize((20, 28)),
        transforms.ToTensor(),
    ])

    if args.from_stdin:
        img = predict_from_stdin()
    elif args.img_path:
        img = predict_from_file(args.img_path)
    else:
        print("-1")
        sys.exit(1)

    input_tensor = transform(img).unsqueeze(0)
    with torch.no_grad():
        output = model(input_tensor)
        pred = output.argmax(dim=1).item()
        print(pred)
