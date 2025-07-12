test_case1 = {
    "name": "Beverages, drinks and snacks",
    "table_objects": ["apple", "fanta_soda", "orange", "cookies", "chips"],
    "shelves": {0: ["tea_lipton", "coke_soda"], 1: ["banana"], 2: ["cereal"]},
    "anwser": {
        0: {"category": "beverages", "objects_to_add": ["fanta_soda"]},
        1: {"category": "fruits", "objects_to_add": ["apple", "orange"]},
        2: {"category": "snacks", "objects_to_add": ["cookies", "chips"]},
    },
}
test_case2 = {
    "name": "Sweets, utencils and sports with empty",
    "table_objects": ["apple", "cookies", "fork", "spoon", "tenis_ball"],
    "shelves": {0: ["tea_lipton"], 1: ["bowl"], 2: []},
    "anwser": {
        0: {"category": "sweets", "objects_to_add": ["apple", "cookies"]},
        1: {"category": "utencils", "objects_to_add": ["fork", "spoon"]},
        2: {"category": "sports", "objects_to_add": ["tenis_ball"]},
    },
}
test_case3 = {
    "name": "Beverages and utencils",
    "table_objects": ["coke_soda", "fanta_soda", "fork"],
    "shelves": {
        0: ["tea_lipton"],
        1: ["spoon"],
    },
    "anwser": {
        0: {"category": "beverages", "objects_to_add": ["apple", "cookies"]},
        1: {"category": "utencils", "objects_to_add": ["fork"]},
    },
}
