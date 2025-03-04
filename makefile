PYTHON_VERSION=3.11.2
VENV_NAME=env

.PHONY: setup install

# Install the venv
setup:
	python -m venv --system-site-packages $(VENV_NAME)

install:
	$(VENV_NAME)/bin/poetry install

# Cleanup environment
clean:
	rm -rf $(VENV_NAME)