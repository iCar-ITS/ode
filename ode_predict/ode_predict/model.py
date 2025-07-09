import torch
import torch.nn as nn
from torchvision import models

class ODEModel_ResNet(nn.Module):

    def __init__(self):
        super(ODEModel_ResNet, self).__init__()
        resnet_model = models.resnet18(weights=models.ResNet18_Weights.IMAGENET1K_V1) 
        self.backbone = nn.Sequential(*list(resnet_model.children())[:-2])  # input:output = 32:1
        # print(self.backbone)
        

        self.neck = nn.Sequential(
            nn.Conv2d(512, 512, kernel_size=5, padding=2),
            nn.Conv2d(512, 1024, kernel_size=1,),
            nn.ReLU(),
            # nn.Conv2d(1024, 1024, kernel_size=3, padding=2),
            # nn.Conv2d(1024, 2048, kernel_size=1,),
            # nn.ReLU(),
            nn.Conv2d(1024, 1024, kernel_size=3, padding=2),
            nn.Conv2d(1024, 4096, kernel_size=1,),
            nn.ReLU(),
        )

        self.feature_dim = 4096+2 # Dimensi fitur dari neck

        self.head = nn.Sequential(
            nn.Linear(self.feature_dim, 2048),
            nn.ReLU(),
            nn.Linear(2048, 1024),
            nn.ReLU(),
            nn.Linear(1024, 1024),
            nn.ReLU(),
            nn.Linear(1024, 1),
            nn.ReLU()
        )

    def forward(self, images, xywh_list):
        """
        images      : Tensor of shape (B, 3, H, W)
        xywh_list : List of B tensors, each with shape (Ni, 4)
        """

        """
        Notes
        Ni : Jumlah titik pada gambar ke-i
        B  : Jumlah gambar dalam batch
        C  : Jumlah channel fitur (2048 untuk ResNet50)
        H  : Tinggi gambar asli
        W  : Lebar gambar asli
        Hf : Tinggi feature map (setelah backbone)
        Wf : Lebar feature map (setelah backbone)
        """

        B, _, H, W = images.shape
        feat = self.backbone(images)    # (B, C, Hf, Wf)
        feat = self.neck(feat)          # (B, 128, Hf, Wf)
        _, C, Hf, Wf = feat.shape

        # Skala koordinat dari gambar asli ke feature map
        scale_x = Wf / W
        scale_y = Hf / H

        outputs = []

        for i in range(B):
            points = xywh_list[i][:,:2]  # (Ni, 2)
            if points.numel() == 0:
                outputs.append(torch.empty(0, device=images.device))
                continue

            px = (points[:,0] * scale_x).long().clamp(0, Wf - 1)
            py = (points[:,1] * scale_y).long().clamp(0, Hf - 1)

            # Ambil fitur dari koordinat
            features = feat[i, :, py, px]        # (C, Ni) 
            features = features.permute(1, 0)    # (Ni, C)

            # append the coordinates to the features
            wh = xywh_list[i][:, 2:4] # (Ni, 2)
            features = torch.cat((features, wh), dim=1)

            # Check if the feature dimension matches the expected dimension
            if features.size(1) != self.feature_dim:
                raise ValueError(f"Feature dimension mismatch: expected {self.feature_dim}, got {features.size(1)}")

            pred = self.head(features).view(-1)  # (Ni,)
            outputs.append(pred)

        return outputs
    

