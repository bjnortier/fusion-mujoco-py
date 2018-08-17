import itertools as it

def make_combinations(all_values):
    var_value_tuples = []
    for var, values in all_values:
        tuples = []
        for value in values:
            tuples.append((var, value))
        var_value_tuples.append(tuples)
    # print(var_value_tuples)
    combinations = it.product(*var_value_tuples)
    return list(combinations)
