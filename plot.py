import torch
import matplotlib.pyplot as plt

# Carica il chunk salvato
chunk = torch.load("/home/ales/lerobot/chunks/chunk_00010.pt")  # shape: (1, chunk_size, action_dim)
chunk = chunk[0]  # ora shape: (chunk_size, action_dim)
print("Chunk shape:", chunk.shape)
plt.figure()
for i in range(chunk.shape[1]):
    plt.plot(chunk[:, i].cpu().numpy(), label=f"action_{i}")
plt.title("Chunk 0 - tutte le azioni")
plt.grid(True)
plt.legend()
plt.show()

