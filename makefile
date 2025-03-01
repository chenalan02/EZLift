PYTHON_VERSION=3.9.18
VENV_NAME=myenv

.PHONY: setup install activate

# Install the required Python version using pyenv
setup:
	pyenv install -s $(PYTHON_VERSION)
	pyenv virtualenv -f $(PYTHON_VERSION) $(VENV_NAME)
	pyenv activate $(VENV_NAME)
	pip install --upgrade pip
	pip install poetry

# Install dependencies with poetry
install:
	poetry install

# Activate the virtual environment
activate:
	pyenv activate $(VENV_NAME)

# Deactivate the virtual environment
deactivate:
	pyenv deactivate

# Cleanup environment
clean:
	pyenv uninstall -f $(VENV_NAME)
	rm -rf .venv