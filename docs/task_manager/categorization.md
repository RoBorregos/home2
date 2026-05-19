# Object Categorization and Shelf Assignment

This document explains the logic behind the `categorize_objects` method in the HRI Task Manager, which is responsible for organizing objects from a table onto available shelves based on their semantic categories.

## Overview

The categorization process follows a deterministic approach to ensure that objects are placed on shelves that already contain similar items, or distributed logically when new categories are introduced.

## Categorization Logic

The system uses a two-tiered approach for identifying the category of an object:

1.  **Direct Lookup:** The object name is cross-referenced against a pre-defined mapping in `frida_constants/data/objects.json`.
2.  **Semantic Similarity:** If no direct match is found, the system uses an embeddings service to find the closest known object and adopts its category.

## Shelf Assignment Algorithm

When assigning `table_objects` to `shelves`, the following priority rules are applied:

### 1. Existing Category Match
If a category of a table object is already present on a shelf (because of objects already stored there), the new object is assigned to that same shelf.

### 2. Empty Shelf Distribution
If a category is "new" (not currently on any shelf), the system looks for empty shelves.
- If multiple empty shelves are available, new categories are distributed across them in a round-robin fashion.
- This ensures that empty space is utilized before crowding existing shelves.

### 3. Overflow Distribution (Avoiding "Miscellaneous")
If there are no empty shelves, or if there are more new categories than empty shelves, the system distributes the new categories across the available shelves (starting with empty ones first) in a round-robin fashion.
- Unlike previous versions, the system **avoids the "miscellaneous" label**.
- Instead, it maintains the specific names of all categories present on a shelf.
- A shelf can have multiple categories (e.g., "fruit food"), which are returned as a list and used by the robot to describe the shelf's contents.

## API Compatibility

The `categorize_objects` method returns:
- `Status`: Execution status.
- `old_api`: A dictionary mapping shelf levels to a space-separated string of categories (for legacy support).
- `objects_to_add`: A dictionary mapping shelf levels to the list of specific table objects to be placed there.
- `categorized_shelves`: A dictionary mapping shelf levels to a list of all categories currently on that shelf.

## Testing

Test cases are defined in `hri/packages/nlp/test/categorize_objects.json`. These tests verify:
- Correct categorization of common objects.
- Proper distribution across empty shelves.
- Correct handling of multiple categories per shelf when space is limited.
- Consistency between `objects_to_add` and the updated `categorized_shelves`.
