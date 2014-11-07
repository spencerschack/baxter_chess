#! /usr/bin/env python

# This class uses the Baxter's sensors and the output of the piece
# finder node to determine whether a move has been made by the opponent.
class Watch:

	def __init__(self):
		self.state = 'initializing'
		while True:
			# Calls the method 'state_<state>'
			getattr(self, 'state_' + self.state)()

	def state_initializing():
		pass

