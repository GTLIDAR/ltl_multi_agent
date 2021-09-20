# Specify a transition system state
# Used to describe an agent state within a TS (transition system) graph.
# Number of dimensions within each TS states can vary depending of the
# defined TS.

# TS (Transition System) state. A TS state is composed of an
# array of states from at least 1 transition model or from the
# product of more transition models.
string[] states

# Name of each state dimension. State should always be ordered the same
# way, but this field can help to disambiguate if needed.
string[] state_dimension_names
