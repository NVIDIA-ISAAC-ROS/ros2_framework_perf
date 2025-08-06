# This message contains common metadata about a message.

# Message timestamp
builtin_interfaces/Time publish_timestamp

# Publish topic name
string topic_name

# Message type
string type

# Unique identifier for the message
string identifier

# Sequence number of the message
uint64 sequence_number

# Name of the node that created the message
string originator

# List of parent message identifiers that this message depends on
string[] parent_messages