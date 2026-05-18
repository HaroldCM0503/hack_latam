import torch
import numpy as np
from config import TrainingConfig
from model import CSIAnomalyDetector
import os

def load_model(checkpoint_path: str, device: torch.device):
    """
    Loads the trained model from a checkpoint.
    """
    if not os.path.exists(checkpoint_path):
        raise FileNotFoundError(f"Checkpoint not found at {checkpoint_path}")
        
    checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
    config = checkpoint['config']
    
    model = CSIAnomalyDetector.from_config(config)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.to(device)
    model.eval()
    
    return model, config

def infer(model: torch.nn.Module, data: np.ndarray, device: torch.device):
    """
    Runs inference on new FFT timeframe data.
    Args:
        model: The loaded PyTorch model.
        data: A numpy array of shape (num_samples, input_size).
              Ensure this is preprocessed/normalized the same way as training data.
        device: The device to run inference on.
    Returns:
        np.ndarray: Predicted coordinates (x, y).
    """
    tensor_data = torch.tensor(data, dtype=torch.float32).to(device)
    
    with torch.no_grad():
        transit_prob, speed_pred = model(tensor_data)
        
    return transit_prob.cpu().numpy(), speed_pred.cpu().numpy()

if __name__ == "__main__":
    # Example usage
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    config = TrainingConfig()
    checkpoint_path = os.path.join(config.checkpoint_dir, config.best_model_name)
    
    try:
        model, loaded_config = load_model(checkpoint_path, device)
        print("Model loaded successfully.")
        
        # Generate some fake test data for inference
        print("Running inference on synthetic test data...")
        test_samples = 5
        dummy_data = np.random.randn(test_samples, loaded_config.input_size)
        
        predictions = infer(model, dummy_data, device)
        
        for i in range(test_samples):
            print(f"Sample {i+1}: Predicted Coordinates = {predictions[i]}")
            
    except FileNotFoundError as e:
        print(e)
        print("Please run train.py first to generate a checkpoint.")
