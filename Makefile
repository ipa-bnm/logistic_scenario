include $(shell rospack find mk)/cmake.mk

regen:
	rosrun bride_compilers m2t -o logistic_scenario -p logistic_scenario model/*.ros_coordinator