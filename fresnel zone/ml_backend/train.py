import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from config import TrainingConfig
from dataset import CSIFFTDataset, generate_fresnel_data
from model import CSIAnomalyDetector

def get_device():
    """Returns the best available device for PyTorch."""
    if torch.cuda.is_available():
        return torch.device("cuda")
    elif torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")

def train_model(config: TrainingConfig, train_loader: DataLoader, val_loader: DataLoader):
    device = get_device()
    print(f"Training on device: {device}")
    
    # Initialize model, loss function, and optimizer
    model = CSIAnomalyDetector.from_config(config).to(device)
    criterion_bce = nn.BCELoss()
    criterion_mse = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=config.learning_rate, weight_decay=config.weight_decay)
    
    best_val_loss = float('inf')
    best_model_path = os.path.join(config.checkpoint_dir, config.best_model_name)
    
    for epoch in range(config.num_epochs):
        # --- Training Phase ---
        model.train()
        train_loss = 0.0
        train_total = 0
        
        for batch_idx, (batch_data, batch_labels) in enumerate(train_loader):
            batch_data, batch_labels = batch_data.to(device), batch_labels.to(device)
            
            optimizer.zero_grad()
            transit_prob, speed_pred = model(batch_data)
            
            transit_target = batch_labels[:, 0:1]
            speed_target = batch_labels[:, 1:2]
            
            loss_bce = criterion_bce(transit_prob, transit_target)
            loss_mse = criterion_mse(speed_pred, speed_target)
            loss = loss_bce + loss_mse
            
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item() * batch_data.size(0)
            train_total += batch_labels.size(0)
            
            if (batch_idx + 1) % max(1, len(train_loader) // 5) == 0:
                print(f"  [Epoch {epoch+1}] Batch {batch_idx+1}/{len(train_loader)} - Batch Loss: {loss.item():.4f}")
                
        epoch_train_loss = train_loss / train_total
        
        # --- Validation Phase ---
        model.eval()
        val_loss = 0.0
        val_total = 0
        
        with torch.no_grad():
            for batch_data, batch_labels in val_loader:
                batch_data, batch_labels = batch_data.to(device), batch_labels.to(device)
                
                transit_prob, speed_pred = model(batch_data)
                
                transit_target = batch_labels[:, 0:1]
                speed_target = batch_labels[:, 1:2]
                
                loss_bce = criterion_bce(transit_prob, transit_target)
                loss_mse = criterion_mse(speed_pred, speed_target)
                loss = loss_bce + loss_mse
                
                val_loss += loss.item() * batch_data.size(0)
                val_total += batch_labels.size(0)
                
        epoch_val_loss = val_loss / val_total
        
        print(f"Epoch {epoch+1}/{config.num_epochs} | "
              f"Train Loss (MSE): {epoch_train_loss:.4f} | "
              f"Val Loss (MSE): {epoch_val_loss:.4f}")
        
        # Checkpoint if validation loss improves
        if epoch_val_loss < best_val_loss:
            best_val_loss = epoch_val_loss
            print(f" -> Saving new best model to {best_model_path}")
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'val_loss': best_val_loss,
                'config': config
            }, best_model_path)

if __name__ == "__main__":
    # Example usage with synthetic data
    cfg = TrainingConfig()
    
    print("Generating synthetic data...")
    data, labels = generate_fresnel_data(num_samples=2000)
    
    # Create dataset
    full_dataset = CSIFFTDataset(data, labels)
    
    # Split into train and validation sets (80% / 20%)
    train_size = int(0.8 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=cfg.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=cfg.batch_size, shuffle=False)
    
    print("Starting training...")
    train_model(cfg, train_loader, val_loader)
    print("Training complete.")
