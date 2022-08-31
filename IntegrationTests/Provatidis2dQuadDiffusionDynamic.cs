using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;

namespace ConvectionDiffusionTest
{
    public static class Provatidis2dQuadDiffusionDynamic
    {
		//dofs:   1,   2,   4,   5,   7,   8
		private static double[] prescribedSolution = new double[] { 150, 200, 150, 200, 150, 200 };

		public static Model CreateModel()
        {
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new Node[]
			{
				new Node(id: 0, x: 0.0, y: 0.0),
				new Node(id: 1, x: 1.0, y: 0.0),
				new Node(id: 2, x: 2.0, y: 0.0),
				new Node(id: 3, x: 0.0, y: 1.0),
				new Node(id: 4, x: 1.0, y: 1.0),
				new Node(id: 5, x: 2.0, y: 1.0),
				new Node(id: 6, x: 0.0, y: 2.0),
				new Node(id: 7, x: 1.0, y: 2.0),
				new Node(id: 8, x: 2.0, y: 2.0)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}
			var material = new ConvectionDiffusionProperties(capacityCoeff: 1d, diffusionCoeff: 1d, convectionCoeff: new[] {0d, 0d} , dependentSourceCoeff:0d, independentSourceCoeff: 0d);
			var elementFactory = new ConvectionDiffusionElement2DFactory(commonThickness: 1.0, material);
			var elements = new ConvectionDiffusionElement2D[]
			{
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[0], nodes[1], nodes[4], nodes[3] }),
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[1], nodes[2], nodes[5], nodes[4] }),
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[3], nodes[4], nodes[7], nodes[6] }),
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[4], nodes[5], nodes[8], nodes[7] })
			};

			for (int i = 0; i < elements.Length; ++i)
			{
				elements[i].ID = i;
				model.ElementsDictionary[i] = elements[i];
				model.SubdomainsDictionary[0].Elements.Add(elements[i]);
			}

			model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
				new[]
				{
					new NodalUnknownVariable(model.NodesDictionary[0], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[3], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable, 100d),
				},
				new[]
				{
					new NodalUnknownVariableFlux(model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable, 25d),
					new NodalUnknownVariableFlux(model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariableFlux(model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable, 25d),
				}
			));

			return model;
		}

		public static void CheckResults(double[] numericalSolution)
		{
			if (numericalSolution.Length != prescribedSolution.Length)
			{
				Console.WriteLine("Array Lengths do not match");
				return;
			}

			var isAMatch = true;
			for (int i = 0; i < numericalSolution.Length; i++)
			{
				var error = Math.Abs((prescribedSolution[i] - numericalSolution[i]) / prescribedSolution[i]);
				Console.WriteLine("Numerical: {0} \tPrescribed: {1} \tError: {2}", numericalSolution[i], prescribedSolution[i], error.ToString("E10"));
				if (error > 1E-6)
				{
					isAMatch = false;
					break;
				}
			}
			if (isAMatch == true)
			{
				Console.WriteLine("MSolve Solution matches prescribed solution");
				Console.WriteLine("Test Passed!");
			}
			else
			{
				Console.WriteLine("MSolve Solution does not match prescribed solution");
				Console.WriteLine("Test Failed!");
			}
		}


	}
}
