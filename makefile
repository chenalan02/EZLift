PYTHON_VERSION=3.11.2
VENV_NAME=env

.PHONY: setup install activate

# Install the venv
setup:
	python$(PYTHON_VERSION) -m pip install --upgrade pip
	curl -sSL https://install.python-poetry.org | python3 -
	poetry config virtualenvs.in-project true
	python -m venv --system-site-packages $(VENV_NAME)

install:
	$(VENV_NAME)/bin/poetry install

# Cleanup environment
clean:
	rm -rf $(VENV_NAME)