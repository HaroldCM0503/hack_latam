import os
import torch
import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS
from torch.utils.data import DataLoader, random_split

from config import TrainingConfig
from dataset import CSIFFTDataset
from model import CSIAnomalyDetector
from train import train_model, get_device
from infer import load_model, infer

app = Flask(__name__)
CORS(app) # Enable CORS for the frontend

# Global model state
device = get_device()
config = TrainingConfig()
checkpoint_path = os.path.join(config.checkpoint_dir, config.best_model_name)
current_model = None
optimizer = None
criterion = None

def init_model():
    global current_model, optimizer, criterion
    import torch.optim as optim
    import torch.nn as nn
    
    try:
        current_model, _ = load_model(checkpoint_path, device)
        print(f"Loaded existing model from {checkpoint_path}")
    except FileNotFoundError:
        print("No existing model found. Needs training.")
        from model import CSIAnomalyDetector
        current_model = CSIAnomalyDetector.from_config(config).to(device)
        
    optimizer = optim.Adam(current_model.parameters(), lr=config.learning_rate, weight_decay=config.weight_decay)
    criterion = nn.MSELoss()

@app.route('/train', methods=['POST'])
def handle_train():
    global current_model
    
    payload = request.json or {}
    
    if 'data' in payload and 'labels' in payload:
        # Train on frontend data
        data = np.array(payload['data'], dtype=np.float32)
        labels = np.array(payload['labels'], dtype=np.float32)
        print(f"Received training data from UI: {data.shape} samples")
    else:
        # Fall back to massive backend simulation (fake live training)
        from dataset import generate_fresnel_data
        # Generate 20,000 samples by default for a proper proof of concept
        num_samples = payload.get('num_samples', 20000)
        data, labels = generate_fresnel_data(num_samples=num_samples)
        
    if len(data) == 0:
        return jsonify({"error": "Empty dataset"}), 400
        
    print(f"Received training data: {data.shape} samples")
    
    # Create dataset and dataloaders
    full_dataset = CSIFFTDataset(data, labels)
    
    # Split into train and validation sets (80% / 20%)
    train_size = int(0.8 * len(full_dataset))
    val_size = len(full_dataset) - train_size
    
    # Handle small datasets gracefully
    if train_size == 0 or val_size == 0:
        train_dataset = full_dataset
        val_dataset = full_dataset # Use same for val if too small
    else:
        train_dataset, val_dataset = random_split(full_dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_dataset, batch_size=config.batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=config.batch_size, shuffle=False)
    
    print("Starting training...")
    # Overwrite epochs to a smaller number for faster UI testing if needed,
    # but we'll stick to config for now.
    train_model(config, train_loader, val_loader)
    
    # Reload the model after training
    init_model()
    
    return jsonify({"message": "Training complete"}), 200

@app.route('/predict', methods=['POST'])
def handle_predict():
    if current_model is None:
        return jsonify({"error": "Model not trained yet"}), 400
        
    payload = request.json
    if not payload or 'features' not in payload:
        return jsonify({"error": "Missing features in payload"}), 400
        
    # Expecting features to be a single sample or batch
    features = np.array(payload['features'], dtype=np.float32)
    
    # If 1D, reshape to 2D
    if len(features.shape) == 1:
        features = features.reshape(1, -1)
        
    # Set to eval mode for clean inference (disables dropout)
    current_model.eval()
    transit_prob, speed_pred = infer(current_model, features, device)
    
    # We only care about the first prediction if we sent 1 sample
    is_transit = float(transit_prob[0][0]) > 0.5
    speed = float(speed_pred[0][0])
    
    return jsonify({"transit": is_transit, "speed": speed}), 200

@app.route('/train_live', methods=['POST'])
def handle_train_live():
    global current_model, optimizer, criterion
    
    payload = request.json
    if not payload or 'features' not in payload or 'labels' not in payload:
        return jsonify({"error": "Missing features or labels"}), 400
        
    features = np.array(payload['features'], dtype=np.float32)
    labels = np.array(payload['labels'], dtype=np.float32)
    
    if len(features.shape) == 1:
        features = features.reshape(1, -1)
    if len(labels.shape) == 1:
        labels = labels.reshape(1, -1)
        
    tensor_features = torch.tensor(features).to(device)
    tensor_labels = torch.tensor(labels).to(device)
    
    current_model.train()
    optimizer.zero_grad()
    outputs = current_model(tensor_features)
    loss = criterion(outputs, tensor_labels)
    loss.backward()
    optimizer.step()
    
    return jsonify({"loss": float(loss.item())}), 200

if __name__ == '__main__':
    init_model()
    # Run on port 5000
    app.run(host='0.0.0.0', port=5000, debug=True)
