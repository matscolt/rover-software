import tritonclient.http as httpclient
from tritonclient.utils import InferenceServerException

# --- Connection Details ---
# Replace with your Triton server's URL and port if different
TRITON_SERVER_URL = "localhost:8000"

def get_available_models(triton_url):
    """
    Connects to a Triton Inference Server and retrieves the list of available models.

    Args:
        triton_url (str): The URL of the Triton Inference Server.

    Returns:
        list: A list of dictionaries, where each dictionary contains information
              about an available model. Returns an empty list if there's an error.
    """
    try:
        # Create a Triton client
        triton_client = httpclient.InferenceServerClient(url=triton_url)

        # Get the model repository index
        repository_index = triton_client.get_model_repository_index()

        return repository_index

    except InferenceServerException as e:
        print(f"Error connecting to Triton or getting model repository: {e}")
        return []

if __name__ == "__main__":
    print(f"Querying Triton server at: {TRITON_SERVER_URL}")
    available_models = get_available_models(TRITON_SERVER_URL)

    if available_models:
        print("\n--- Available Models ---")
        for model in available_models:
            print(f"  - Model Name: {model['name']}")
            if 'version' in model:
                print(f"    Version: {model['version']}")
            if 'state' in model:
                print(f"    State: {model['state']}")
            if 'reason' in model and model['reason']:
                print(f"    Reason: {model['reason']}")
            print("-" * 20)
    else:
        print("No models found or could not connect to the server.")
