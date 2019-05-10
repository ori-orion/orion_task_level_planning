Task Level Planning
===================

All code for ORIon task level planning.

To create a parser for the GPSR task, please do the entering the following command:

`grako --whitespace "\t\n.'," -o <OUTPUT_FILE> <GRAKO_GRAMMAR_FILE>`

The whitespace has to be set like this to avoid punctuation and escape characters

Then, please use GPSR_utils to use the parser with `parse_string`. This is because grako doesn't deal with everything we want...
