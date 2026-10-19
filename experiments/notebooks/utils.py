
def print_latex_table(
    df,
    columns=None,
    headers=None,
    include_header=False,
    include_index=False,
    significant_figures=4,
    indent=0,
):
    """
    Print a pandas DataFrame as an aligned LaTeX table.

    Parameters
    ----------
    df : pandas.DataFrame
        DataFrame to print.
    columns : list, optional
        Columns to display, in the desired order.
    headers : list, optional
        Custom column headers.
    include_header : bool
        Whether to print the header row.
    include_index : bool
        Whether to include the DataFrame index.
    significant_figures : int
        Number of significant figures for numeric values.
        Scientific notation is never used.
    indent : int
        Number of spaces before each row.
    """

    import pandas as pd
    import numpy as np

    if significant_figures < 1:
        raise ValueError("significant_figures must be at least 1")

    # Select columns and preserve their order
    data = df[columns].copy() if columns is not None else df.copy()

    if include_index:
        data.insert(0, "Index", df.index)

    # Format values without scientific notation
    def format_value(value):
        if pd.isna(value):
            return ""

        if isinstance(value, (int, np.integer)):
            return str(value)

        if isinstance(value, (int, float, np.integer, np.floating)):
            if not np.isfinite(value):
                return str(value)

            if value == 0:
                return "0" if significant_figures == 1 else (
                    "0." + "0" * (significant_figures - 1)
                )

            decimals = significant_figures - 1 - int(
                np.floor(np.log10(abs(value)))
            )

            if decimals >= 0:
                return f"{value:.{decimals}f}"

            rounded = round(value, decimals)
            return f"{rounded:.0f}"

        return str(value)

    rows = [
        [format_value(value) for value in row]
        for row in data.itertuples(index=False, name=None)
    ]

    # Optionally add a header row
    if include_header:
        if headers is None:
            header_row = [str(col) for col in data.columns]
        else:
            header_row = [str(h) for h in headers]
            if len(header_row) != len(data.columns):
                raise ValueError(
                    "headers must match the number of columns"
                )

        rows.insert(0, header_row)

    if not rows:
        return ""

    # Compute maximum width of each column
    widths = [
        max(len(row[j]) for row in rows)
        for j in range(len(rows[0]))
    ]

    # Identify numeric columns
    numeric_cols = [
        pd.api.types.is_numeric_dtype(data[col].dtype)
        for col in data.columns
    ]

    # Build aligned LaTeX rows
    lines = []

    for i, row in enumerate(rows):
        cells = []

        for j, value in enumerate(row):
            if include_header and i == 0:
                cells.append(value.ljust(widths[j]))
            elif numeric_cols[j]:
                cells.append(value.rjust(widths[j]))
            else:
                cells.append(value.ljust(widths[j]))

        lines.append(
            " " * indent + " & ".join(cells) + r" \\"
        )

    result = "\n".join(lines)
    print(result)
    return result

BKR = {
    "md1a": 308.567,
    "md2a": 525.598,
    "md3a": 678.766,
    "md4a": 823.440,
    "md5a": 979.404,
    "md6a": 1189.143,
    "md7a": 399.786,
    "md8a": 718.072,
    "md9a": 876.011,
    "md10a": 1143.003,
    "md1b": 255.980,
    "md2b": 460.163,
    "md3b": 605.351,
    "md4b": 773.215,
    "md5b": 878.682,
    "md6b": 1052.225,
    "md7b": 325.935,
    "md8b": 627.347,
    "md9b": 886.537,
    "md10b": 1120.415
}
