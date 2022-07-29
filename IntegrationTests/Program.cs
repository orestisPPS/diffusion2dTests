﻿using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.Solvers.Direct;
using MGroup.Solvers.Iterative;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Line;
using System.Collections.Generic;

namespace ConvectionDiffusionTest

{
    class Program
    {
        static void Main(string[] args)
        {
            //1D
            //RodDiffusionTest();
            //RodConvectionDiffusionTest();

            //2D
            //Provatidis2dDiffusionSteadyStateTest();
            //Provatidis2dDiffusionDynamicTest();
            //Reddy2dDiffusionSteadyStateTest();
            //Comsol2DConvectionDiffusionSteadyStateTest();
            //Comsol2DConvectionDiffusionDynamicTest();
            Comsol2DConvectionDiffusionProductionDynamicTest();

            //3D
            //Comsol3DConvectionDiffusionSteadyStateHexaTest
            //Comsol3DConvectionDiffusionDynamicHexaTest();
        }

        static void RodDiffusionTest()
        {
            var model = DiffusionRodCengel.CreateModel();
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            //var analyzer = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000);
            //analyzer.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            //var analyzer = dynamicAnalyzerBuilder.Build();

            var analyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
                        {
                            (model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable),
                        };
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            analyzer.Initialize();
            analyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            DiffusionRodCengel.CheckResults(log.DOFValues[watchDofs[0].node, watchDofs[0].dof]);
            //Console.WriteLine("break");
        }

        static void RodConvectionDiffusionTest()
        {
            var model = ConvectionDiffusionRodZienkiewicz.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            //var analyzer = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000);
            //analyzer.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            //var analyzer = dynamicAnalyzerBuilder.Build();

            var analyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[3], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[4], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[9], ConvectionDiffusionDof.UnknownVariable)
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            analyzer.Initialize();
            analyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            ConvectionDiffusionRodZienkiewicz.CheckResults(numericalSolution);
            //Console.WriteLine("break");
        }

        static void Provatidis2dDiffusionSteadyStateTest()
        {
            var model = Provatidis2DQuadSteadyStateTest.CreateModel();
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000);
            dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            /*            var watchDofs = new List<(INode node, IDofType dof)>()
                        {
                            (model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable),
                            (model.NodesDictionary[3], ConvectionDiffusionDof.UnknownVariable),
                            (model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable),
                            (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),
                            (model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable),
                            (model.NodesDictionary[9], ConvectionDiffusionDof.UnknownVariable)
                        };*/

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable),

            };
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Provatidis2DQuadSteadyStateTest.CheckResults(numericalSolution);
            //Console.WriteLine("break");
        }

        static void Reddy2dDiffusionSteadyStateTest()
        {
            var model = Reddy2dHeatDiffusionTest.CreateModel();
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 10000);
            dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();
            
            var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),

            };
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Reddy2dHeatDiffusionTest.CheckResults(numericalSolution);
            //Console.WriteLine("break");
        }

        static void Provatidis2dDiffusionDynamicTest()
        {
            var model = Provatidis2dQuadDiffusionDynamic.CreateModel();
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000);
            dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[4], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable),

            };
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Provatidis2dQuadDiffusionDynamic.CheckResults(numericalSolution);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();
        }

        static void Comsol2DConvectionDiffusionSteadyStateTest()
        {
            var model = Comsol2DConvectionDiffusionStadyState.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var analyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[11], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            analyzer.Initialize();
            analyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Comsol2DConvectionDiffusionStadyState.CheckResults(numericalSolution);
        }

        static void Comsol2DConvectionDiffusionDynamicTest()
        {
            var model = Comsol2DConvectionDiffusionDynamic.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep : 0.1, totalTime : 1, bdfOrder : 2);
            //var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 2);
            //dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
/*                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable),*/
                (model.NodesDictionary[11], ConvectionDiffusionDof.UnknownVariable)
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Comsol2DConvectionDiffusionDynamic.CheckResults(numericalSolution);
        }

        static void Comsol3DConvectionDiffusionStadyStateHexaTest()
        {
            var model = Comsol3DConvectionDiffusionStadyStateHexa.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var analyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            analyzer.Initialize();
            analyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Comsol3DConvectionDiffusionStadyStateHexa.CheckResults(numericalSolution);
        }

        static void Comsol3DConvectionDiffusionDynamicHexaTest()
        {
            var model = Comsol3DConvectionDiffusionDynamicHexa.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.1, totalTime: 10, bdfOrder: 5);
            //var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 2);
            //dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable),
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Comsol3DConvectionDiffusionDynamicHexa.CheckResults(numericalSolution);
        }
        
        static void Comsol2DConvectionDiffusionProductionDynamicTest()
        {
            var model = Comsol2DConvectionDiffusioProductionDynamic.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.1, totalTime: 10, bdfOrder: 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),  //[1,1]
                (model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable), //[1,2]
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),  //[2,1]
                (model.NodesDictionary[11], ConvectionDiffusionDof.UnknownVariable)  //[2,2]
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Comsol2DConvectionDiffusioProductionDynamic.CheckResults(numericalSolution);
        }
    }
}
