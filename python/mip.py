import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB
from settings import *
from common import *


def ip_solve():
	model = gp.Model('MRMP')
	model.Params.LogToConsole = 1
	model.Params.TimeLimit = 600
	model.Params.MIPGap = 1e-2
	model.Params.OptimalityTol = 1e-2
	dvars = []
	vars = []
	yvars = []

	for i in range(num_bins):
		xi = []
		yi = []
		for j in range(types):
			xij = model.addVar(vtype=GRB.BINARY, name="x_"+str(i)+"_"+str(j))
			yij = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name="y_"+str(i)+"_"+str(j))
			xi.append(xij)
			yi.append(yij)
			model.addConstr(yij*(1e-3+xij) == 1)
		vars.append(xi)
		yvars.append(yi)

	for k in range(num_station):
		dk = []
		for j in range(types):
			dkj = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name="d_"+str(k)+"_"+str(j))
			dk.append(dkj)
			# discrete.append(True)
		dvars.append(dk)

	# for each bin
	for i in range(num_bins):
		vari = [vars[i][j] for j in range(types)]
		constraint_expr = gp.LinExpr([1 for i in vari], vari)
		model.addLConstr(constraint_expr, GRB.EQUAL, 1)

	# for each type
	for j in range(types):
		varj = [vars[i][j] for i in range(num_bins)]
		constraint_expr = gp.LinExpr([1 for i in varj], varj)
		model.addLConstr(constraint_expr, GRB.GREATER_EQUAL, 1)

	# dkj
	for k in range(num_station):
		for j in range(types):
			var_arr = []
			for i in range(num_bins):
				yij_weighted=model.addVar(lb=0,vtype=GRB.CONTINUOUS)
				model.addConstr(yij_weighted==distance(station_position[k], bin_positions[i])*yvars[i][j])
				var_arr.append(yij_weighted)
			model.addConstr(dvars[k][j]==gp.min_(var_arr))
	
	# objective_expression = gp.LinExpr()
	cost=[prob[k][j] for k in range(num_station) for j in range(types)]
	var_obk=[dvars[k][j] for k in range(num_station) for j in range(types)]
	objective_expression=gp.LinExpr(cost,var_obk)

	model.setObjective(objective_expression, GRB.MINIMIZE)
	model.optimize()
	assignment=[]
	for i in range(num_bins):
		for k in range(types):
			if abs(vars[i][k].x-1)<1e-3:
				assignment.append(k)
				break
			
	
	print('Obj: %g' % model.ObjVal)
	return assignment

if __name__=="__main__":
	assignment=ip_solve()
	print(assignment)
