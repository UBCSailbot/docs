# docs

UBCSailbot software team's documentation.

## Setup

1. Clone repository

    ```
    git clone https://github.com/patrick-5546/notes.git
    ```

2. Install dependencies

    ```
   pip install -r docs/requirements.txt
   ```

    - Can do this in a [Python virtual environment](https://patrick-5546.github.io/notes/reference/python/#virtual-environments)

## Run

### VS Code

1. `CTRL+P` to open Quick Open
2. Run a launch configuration
    - "debug Run Application" runs `mkdocs serve`
    - "debug Launch Application" runs `mkdocs serve` and opens the application in a new Microsoft Edge window

### Command line

```
mkdocs serve
```
