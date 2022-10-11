# docs

UBCSailbot software team's documentation.

## Setup

1. Clone repository

    ```
    git clone https://github.com/patrick-5546/notes.git
    ```

2. Install dependencies

    ```
   pip install -Ur docs/requirements.txt
   ```

    - Can do this in a [Python virtual environment](https://ubcsailbot.github.io/docs/reference/python/virtual-environments/)

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

## Update Dependencies

This site is built using the latest versions of dependencies in [`docs/requirements.txt`](./docs/requirements.txt)
at the time of the most recent commit to the main branch.
To see exactly how the site will look before pushing or merging into the main branch,
ensure your local dependencies are up to date with

```
pip install -Ur docs/requirements.txt --upgrade-strategy eager
```
