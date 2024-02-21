# This script is to check that the policy works well in cpp.
# The script reads observation data from some data file
# And computes and stores the policy output in python
# Compare the action with the original data to check validity of policy
# Recommended to run in python3.8

import torch
import os
import numpy as np

def main():
    # Define the paths to the input text file and the PyTorch model
    current_dir = os.getcwd()
    input_txt_file = os.path.abspath(os.path.join(current_dir + "../../data/data.txt"))
    model_file = os.path.join(current_dir, 'policy_1.pt')
    
    # Load the PyTorch model
    model = torch.jit.load(model_file)

    # Read odd numbered lines from the input text file
    with open(input_txt_file, 'r') as file:
        lines = file.readlines()
        odd_lines = [line.strip() for i, line in enumerate(lines) if i % 2 == 0]

    # Convert odd lines to tensors and pass through the model
    output_tensors = []
    for line in odd_lines:
        # Convert the line to a tensor
        tensor_input = torch.tensor([float(x) for x in line.split()])
        # Pass the tensor through the model
        output_tensor = model(tensor_input)
        output_tensors.append(output_tensor)
    # Write the output tensor data to a new text file
    output_txt_file = "output.txt"
    with open(output_txt_file, 'w') as file:
        for tensor in output_tensors:
            # Write the tensor data (numbers only) to the file
            file.write(' '.join(map(lambda x: f"{x:.6f}", tensor.tolist())) + '\n')

if __name__ == "__main__":
    main()

