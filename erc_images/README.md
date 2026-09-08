# erc_images

Annotated detection images are written here at run time by
`rafeeq_solution/perception_node.py`:

- `shelf_column_<timestamp>.png` — bounding box around the target column's number plate
- `target_book_<timestamp>.png`  — bounding box around the target book
- `red_bin_<timestamp>.png`      — bounding box around the collection bin

Inside the container this directory is `/opt/erc_ws/src/erc_images`, and
`/erc_images` is symlinked to it so the rulebook path also works. It is in the
mounted workspace, so the images appear on the host and survive the container.
