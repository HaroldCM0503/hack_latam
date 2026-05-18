import torch
from torch.utils.data import Dataset
import numpy as np

class CSIFFTDataset(Dataset):
    """
    Dataset for Channel State Information data.
    """
    def __init__(self, data: np.ndarray, labels: np.ndarray):
        self.data = torch.tensor(data, dtype=torch.float32)
        self.labels = torch.tensor(labels, dtype=torch.float32)
        
        # Simple normalization per feature (standard scaling)
        mean = self.data.mean(dim=0, keepdim=True)
        std = self.data.std(dim=0, keepdim=True)
        std[std == 0] = 1.0 
        self.data = (self.data - mean) / std

    def __len__(self):
        return len(self.labels)

    def __getitem__(self, idx):
        return self.data[idx], self.labels[idx]

def generate_fresnel_data(num_samples: int):
    """
    Generates exact mathematical equivalent of the JS Fresnel simulation.
    Returns 500-sample sequences of raw CSI amplitude.
    50% are transits (positive label), 50% are clear (negative label).
    """
    print(f"Generating {num_samples} samples of pure Fresnel math (1D sequences)...")
    C = 299792458
    freq = 23e9
    lam = C / freq
    noise_level = 0.05
    room_w = 4000000.0
    room_h = 3000.0
    tx_x, tx_y = 0.0, 1500.0
    rx_x, rx_y = 4000000.0, 1500.0
    D = np.hypot(rx_x - tx_x, rx_y - tx_y)
    
    seq_len = 500
    dt = 0.0016 # Matches JS: 60FPS (0.016s) * timeScale (0.1)
    
    features = np.zeros((num_samples, seq_len), dtype=np.float32)
    labels = np.zeros((num_samples, 2), dtype=np.float32) # [is_transit, speed]
    
    # Generate labels
    # Half transits, half stationary/empty
    num_transits = num_samples // 2
    
    # --- TRANSITS ---
    # Transits should cross the center line y=1500 somewhere between x=1,000,000 and 3,000,000
    transit_x_crossing = np.random.uniform(1000000, 3000000, num_transits)
    transit_y_crossing = 1500.0
    
    speed = np.random.uniform(2000, 10000, num_transits)
    
    # Angle pointing in any direction, but avoid nearly horizontal (< 0.1 sin)
    angle = np.zeros(num_transits)
    for i in range(num_transits):
        a = np.random.uniform(0, 2 * np.pi)
        while abs(np.sin(a)) < 0.1:
            a = np.random.uniform(0, 2 * np.pi)
        angle[i] = a
        
    vx = np.cos(angle) * speed
    vy = np.sin(angle) * speed
    
    # Start position: rewind from crossing point by half the sequence length
    start_x = transit_x_crossing - vx * (dt * seq_len / 2)
    start_y = transit_y_crossing - vy * (dt * seq_len / 2)
    
    for i in range(num_transits):
        if (i + 1) % max(1, num_transits // 10) == 0:
            print(f"[Generation] Transits: {i+1}/{num_transits} ({(i+1)/num_transits*100:.0f}%)")
            
        curr_x, curr_y = start_x[i], start_y[i]
        for step in range(seq_len):
            curr_x += vx[i] * dt
            curr_y += vy[i] * dt
            
            d_tx = np.hypot(curr_x - tx_x, curr_y - tx_y)
            d_rx = np.hypot(curr_x - rx_x, curr_y - rx_y)
            excess_path = (d_tx + d_rx) - D
            
            attenuation = np.exp(-excess_path / (lam * 3.0))
            phase = (2 * np.pi * excess_path) / lam + np.pi
            
            total_amp = np.sqrt(1 + attenuation**2 + 2 * attenuation * np.cos(phase))
            noise = (np.random.rand() * 2 - 1) * noise_level
            features[i, step] = (total_amp - 1.0) + noise
            
        labels[i, 0] = 1.0
        # Scale speed to roughly [0, 1] range so MSE converges well (divide by 10000 max speed)
        labels[i, 1] = speed[i] / 10000.0

    # --- NEGATIVES ---
    # Stationary objects or objects very far away moving parallel
    for i in range(num_transits, num_samples):
        idx = i - num_transits + 1
        total_neg = num_samples - num_transits
        if idx % max(1, total_neg // 10) == 0:
            print(f"[Generation] Clear Space: {idx}/{total_neg} ({idx/total_neg*100:.0f}%)")
            
        # 80% of negatives are completely stationary (just noise)
        # 20% are moving parallel but far away
        if np.random.rand() < 0.8:
            curr_x = np.random.uniform(0, room_w)
            curr_y = np.random.uniform(0, room_h)
            neg_vx = 0.0
            neg_vy = 0.0
        else:
            curr_x = np.random.uniform(0, room_w)
            curr_y = np.random.choice([np.random.uniform(-1000, 500), np.random.uniform(2500, 4000)])
            neg_vx = np.random.uniform(-1000, 1000)
            neg_vy = np.random.uniform(-100, 100)
            
        for step in range(seq_len):
            curr_x += neg_vx * dt
            curr_y += neg_vy * dt
            
            d_tx = np.hypot(curr_x - tx_x, curr_y - tx_y)
            d_rx = np.hypot(curr_x - rx_x, curr_y - rx_y)
            excess_path = (d_tx + d_rx) - D
            
            attenuation = np.exp(-excess_path / (lam * 3.0))
            phase = (2 * np.pi * excess_path) / lam + np.pi
            
            total_amp = np.sqrt(1 + attenuation**2 + 2 * attenuation * np.cos(phase))
            noise = (np.random.rand() * 2 - 1) * noise_level
            features[i, step] = (total_amp - 1.0) + noise
            
        labels[i, 0] = 0.0
        labels[i, 1] = 0.0
        
    print("Generation complete.")
    return features, labels
