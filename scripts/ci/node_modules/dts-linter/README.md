![Build Status](https://github.com/kylebonnici/dts-linter/actions/workflows/node/badge.svg)

# DTS Linter

A comprehensive DeviceTree linter and formatter that provides syntax checking, formatting, and diagnostic capabilities for DeviceTree Source (.dts), DeviceTree Source Include (.dtsi), and overlay files.

## Features

- **Formatting**: Automatically format DeviceTree files with proper indentation and style
- **Diagnostics**: Comprehensive syntax and semantic error checking
- **CI/CD Integration**: Designed to work seamlessly with GitHub Actions
- **Batch Processing**: Process multiple files efficiently
- **Diff Output**: Generate patch files for formatting changes

## Installation

```bash
npm install -g --ignore-scripts dts-linter
```

## Usage

### Basic Usage

```bash
# Lint and format specific files
dts-linter --file file1.dts --file file2.dtsi --format

# Auto-discover and process all DTS files in current directory
dts-linter --format

# Apply formatting changes directly to files
dts-linter --formatFixAll
```

### Command Line Options

| Option                  | Type            | Default         | Description                                                                                        |
| ----------------------- | --------------- | --------------- | -------------------------------------------------------------------------------------------------- |
| `--file`                | string          | Auto-discover   | List of input files (can be repeated)                                                              |
| `--cwd`                 | string          | `process.cwd()` | Set the current working directory                                                                  |
| `--include`             | string          | `[]`            | Paths to resolve includes (absolute or relative to CWD, can be repeated)                           |
| `--binding`             | string          | `[]`            | Zephyr binding root directories (absolute or relative to CWD, can be repeated)                     |
| `--logLevel`            | `none\|verbose` | `none`          | Set the logging verbosity                                                                          |
| `--format`              | boolean         | `false`         | Format the specified files (automatically set to true when formatFixAll)                           |
| `--formatFixAll`        | boolean         | `false`         | Apply formatting changes directly to files                                                         |
| `--diagnostics`         | boolean         | `false`         | Show basic syntax diagnostics                                                                      |
| `--diagnosticsFull`     | boolean         | `false`         | Show full diagnostics including semantic analysis (requires --include, --binding for proper usage) |
| `--diagnosticsConfig`   | string          | -               | Path to json file with test case configurations for complex diagnostics                            |
| `--showInfoDiagnostics` | boolean         | `false`         | Show information-level diagnostics                                                                 |
| `--patchFile`           | string          | -               | Write formatting diff output to file                                                               |
| `--outputType`          | string          | `auto`          | Output format type: auto, pretty, annotations, or json                                             |
| `--threads`             | number          | `1`             | Number of parallel LSP instances to use for processing files                                       |
| `--help`                | boolean         | `false`         | Show help information                                                                              |

### Examples

#### Check formatting without making changes

```bash
dts-linter --format --file my-board.dts
```

#### Auto-fix formatting issues

```bash
dts-linter --formatFixAll --file my-board.dts --file my-overlay.dtsi
```

#### Full diagnostic check with include processing

```bash
dts-linter --diagnosticsFull --include ./include --binding ./zephyr/dts/bindings
```

#### Generate diff file for review

```bash
dts-linter --format --patchFile changes.patch
```

#### Use multiple threads for faster processing

```bash
dts-linter --format --diagnostics --threads 4
```

## File Discovery

When no `--file` option is provided, the linter automatically searches for DeviceTree files using the pattern `**/*.{dts,dtsi,overlay}` or `**/*.{dts}` when using `--diagnosticsFull` in the current working directory.

## Performance and Threading

The linter supports parallel processing using multiple LSP instances to improve performance when processing many files:

- Use `--threads N` to specify the number of parallel LSP instances
- Each thread runs its own LSP server instance
- Files are automatically distributed among available threads
- Recommended to use 2-4 threads for optimal performance depending on your system
- Threading is particularly beneficial when processing large numbers of files or when using `--diagnosticsFull`

## Output Formats

### Standard Output

- ✅ Success messages with file counts
- ⚠️ Warnings for formatting issues
- ❌ Errors for syntax problems

### CI/CD Output

When running in CI environments (GitHub Actions, GitLab CI, etc.), the tool automatically formats output using platform-specific annotations:

- `::notice` for information
- `::warning` for formatting issues
- `::error` for syntax errors
- File locations and line numbers included

### JSON

Returns a JSON object of type

```typescript
{
    cwd: string;
    issues: {
        level: string;
        message: string;
        title?: string;
        file?: string;
        startLine?: number;
        startCol?: number;
        endLine?: number;
        endCol?: number;
    }[];
}
```

### Diff Output

When using `--patchFile`, generates unified diff format showing all formatting changes:

```diff
--- a/my-board.dts
+++ b/my-board.dts
@@ -10,7 +10,7 @@
 / {
-	model = "My Board";
+    model = "My Board";
     compatible = "vendor,my-board";
 };
```

## Diagnostic Levels

The linter supports multiple diagnostic severity levels:

- **Error**: Syntax errors that prevent compilation
- **Warning**: Potential issues that should be reviewed
- **Information**: Informational messages (shown with `--showInfoDiagnostics`)
- **Hint**: Style suggestions

## Integration with Language Server

This linter uses the DeviceTree Language Server for accurate parsing and validation. It supports:

- Context-aware validation
- Include path resolution
- Zephyr binding validation
- Workspace-wide analysis

## Exit Codes

- `0`: Success - all files processed without errors
- `1`: Failure - formatting errors or diagnostic issues found

## Requirements

- Node.js 16+

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

Apache License, Version 2.0

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for version history and changes.
