#The robot links for decentralization

LINE_LINKS = {
    "tb3_0": ["tb3_1"],
    "tb3_1": ["tb3_0", "tb3_2"],
    "tb3_2": ["tb3_1", "tb3_3"],
    "tb3_3": ["tb3_2", "tb3_4"],
    "tb3_4": ["tb3_3"]
}
