# MarauderTech 9573 FRC Codebase - 2026 Rebuilt (Rev)

Some really cool robot code, except it's rewritten in python

## Usage

### Prerequisites

- Windows ideally
- Python ~3.14

### 1. Clone the repo

Git clone this repository or similar

### 2. Set up environment

Install the `robotpy` package. This can be done globally in your python install, or locally by creating a venv into `.venv`.

Additionally, tt is possible to install this project as a package, using `pip install .`. This will install the above packages.

Don't forget to ensure your code editor recognizes your environment

**Note**: The vs-code tasks included in this repo assume a windows machine with a venv in `.venv` containing `robotpy` and `ruff`.

### 3. Sync robotpy

Before you can use `robotpy` commands, you must sync `robotpy`'s packages:

```bash
robotpy sync
```

This will download the robot-related packages utilized in this project, allowing for simulation, testing, and intellisense in the codebase.

**Note**: If `robotpy` is not a valid command, verify that you are running it as a script in your selected environment, for instance as `py -m robotpy` for your global python installation. You may also need to select your environment in your code editor if running from there.

### 4. Test, simulate, and deploy

You can run `robotpy` commands as needed to test and deploy the project, such as:

```bash
robotpy # Show robotpy commands
```

Common commands you may use are: `test`, `deploy`, `sim`, and `sync`.

## License

I'm nice so this project is licensed with the MIT license
