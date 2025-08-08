import numpy as np
import tritonclient.http as httpclient

# --- Model and Server Details ---
MODEL_NAME = "IQL_depth_224_224"
MODEL_VERSION = "1"  # Optional: specify a version, or leave as an empty string for latest
TRITON_SERVER_URL = "localhost:8000" # Replace with your server's URL

# --- 1. Create Triton Client ---
# This client is used to send requests to the server.
try:
    triton_client = httpclient.InferenceServerClient(url=TRITON_SERVER_URL)
except Exception as e:
    print(f"Could not create Triton client: {e}")
    # Exit if we can't even create a client
    exit()

# --- 2. Define Model Inputs ---
# We need to create an 'InferInput' object for each input tensor specified
# in the model configuration.

# Input 1: proprioceptive
proprioceptive_input = httpclient.InferInput(
    "proprioceptive",   # Name of the input
    [1, 5],             # Shape of the input tensor (batch size 1, 4 features)
    "FP32"              # Data type
)

# Input 2: image
image_input = httpclient.InferInput(
    "image",                            # Name of the input
    [1, 1, 224, 224],                   # Shape of the input tensor (batch size 1, 1 channel, 224x224)
    "FP32"                              # Data type
)


# --- 3. Create Dummy Input Data ---
# Create NumPy arrays with the correct shape and data type to serve as
# the actual data for the request. For this example, we use random data.
proprioceptive_data = np.random.rand(1, 5).astype(np.float32)
image_data = np.random.rand(1, 1, 224, 224).astype(np.float32)

# --- 4. Set Data for Inputs ---
# Assign the NumPy data to the corresponding 'InferInput' objects.
# The 'set_data_from_numpy' method handles the conversion.
proprioceptive_input.set_data_from_numpy(proprioceptive_data, binary_data=True)
image_input.set_data_from_numpy(image_data, binary_data=True)


# --- 5. Define Model Outputs ---
# Specify which output tensors you want the server to return.
action_output = httpclient.InferRequestedOutput(
    "action",           # Name of the output
    binary_data=True    # Request the output in binary format
)


# --- 6. Assemble the Request ---
# The request is now fully defined with its inputs and requested outputs.
# The following code shows how you would send it to the server.

print(f"Assembling request for model: '{MODEL_NAME}'")
print(f"Input '{proprioceptive_input.name()}' with shape {proprioceptive_data.shape} and dtype {proprioceptive_data.dtype}")
print(f"Input '{image_input.name()}' with shape {image_data.shape} and dtype {image_data.dtype}")
print(f"Requesting output: '{action_output.name()}'")


# --- 7. Send the Inference Request (Example) ---
# This is how you would execute the inference call.
# NOTE: This will fail if you are not running a Triton server with the
# 'IQL_depth' model loaded and ready.

try:
    print("\nAttempting to send inference request (this will fail if server/model is not running)...")
    # The infer method sends the request to the Triton server
    response = triton_client.infer(
        model_name=MODEL_NAME,
        model_version=MODEL_VERSION,
        inputs=[proprioceptive_input, image_input],
        outputs=[action_output]
    )

    # --- 8. Process the Response ---
    # If the request is successful, you can get the results.
    action_result = response.as_numpy("action")
    print(f"\nInference successful!")
    print(f"Received 'action' output with shape: {action_result.shape}")
    print(f"Result: {action_result}")

except Exception as e:
    print(f"\nInference failed: {e}")
