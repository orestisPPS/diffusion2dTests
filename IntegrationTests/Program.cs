using MGroup.Constitutive.ConvectionDiffusion;
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
using System.Diagnostics;
using MGroup.MSolve.AnalysisWorkflow;
using MGroup.NumericalAnalyzers.Staggered;
using MGroup.MSolve.Solution;

namespace ConvectionDiffusionTest

{
    class Program
    {
        static List<(INode node, IDofType dof)>[] watchDofs;
        static LinearAnalyzer[] linearAnalyzers;
        static GenericAnalyzerState[] analyzerStates;
        static Model[] model;
        static double timeStep = 0.1;
        static double totalTime = 10;
        static int currentTimeStep = 0;

        //Staggered testing
        static int bdford = 5;
        static int maxStaggeredSteps = 2;
        static bool useNewmark = false;

        static void Main(string[] args)
        {
            //Mesh thigs
            //var reader = new ComsolMeshReader("../../../Meshes/OfficialMeshCyprus.mphtxt");

            Stopwatch stopwatch = new Stopwatch();
            stopwatch.Start();
            //1D
            //RodDiffusionTest();                                                        //Passed +
            //RodConvectionDiffusionTest();                                              //Borderline fail - Passing 1e-3

            //2D
            //Provatidis2dDiffusionSteadyStateTest();                                   //Passed +
            //Provatidis2dDiffusionDynamicTest();                                       //Wrong Matrices -
            //Reddy2dDiffusionSteadyStateTest();                                        //Passed

            //Comsol2DConvectionDiffusionSteadyStateTest();                             //PASSED 100 FWTIA
            //Comsol2DConvectionDiffusionProductionSteadyStateTest();                   //Passed 1E-3 error x2
            //Comsol2DConvectionDiffusionDynamicTest();                                 //Passed 1E-4 error x2
            //Comsol2DConvectionDiffusionProdDynamicTest();                             //Passed 1E-4 error x2

            //3D
            //Comsol3DConvectionDiffusionStadyStateHexaTest();                          //PASSED 100 FWTIA
            //Comsol3DConvectionDiffusionProductionStadyStateHexaTest();                //PASSED 100 FWTIA x2 + ComsolMesh
            //Comsol3DConvectionDiffusionDynamicHexaTest();                             //PASSED 100 FWTIA
            //Comsol3DConvectionDiffusionProductionDynamicHexaTest();                   //PASSED 100 FWTIA x2

            //Staggered
            //Comsol3DStaggeredStatic();                                                //PASSED
            Comsol3DStaggeredDynamic();


            stopwatch.Stop();
            Console.WriteLine("CPU time : " + stopwatch.ElapsedMilliseconds + "ms");
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

            //var analyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 10);
            //analyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            //var analyzer = analyzerBuilder.Build();

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
            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 10);
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

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep : 0.1, totalTime : 10, bdfOrder : 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),
                (model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable),
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

        static void Comsol2DConvectionDiffusionProductionSteadyStateTest()
        {
            var model = Comsol2DConvectionDiffusionProductionStadyState.CreateModel();
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
            Comsol2DConvectionDiffusionProductionStadyState.CheckResults(numericalSolution);
        }

        static void Comsol2DConvectionDiffusionProdDynamicTest()
        {
            var model = Comsol2DConvectionDiffusioProductionDynamic.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.1, totalTime: 10, bdfOrder: 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();
            //var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 2);
            //dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.25, gamma: 0.5, allowConditionallyStable: true);
            //var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

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

        static void Comsol3DConvectionDiffusionProductionStadyStateHexaTest()
        {
            var model = Comsol3DConvectionDiffusionProductionStStHexa.CreateModelFromComsolFile("../../../Meshes/3d8Hexa.mphtxt"); // 3d8Hexa
            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel, solver);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var analyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable),
                //(model.NodesDictionary[3474], ConvectionDiffusionDof.UnknownVariable), //Finer
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
            Comsol3DConvectionDiffusionProductionStStHexa.CheckResults(numericalSolution);
        }

        static void Comsol3DStaggeredStatic()
        {
            model = new[] {
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Hexa.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d),
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Hexa.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 2d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 2d)
            };

            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!

            var algebraicModel = new[] {
                solverFactory.BuildAlgebraicModel(model[0]),
                solverFactory.BuildAlgebraicModel(model[1])
            };

            var solver = new[] {
                solverFactory.BuildSolver(algebraicModel[0]),
                solverFactory.BuildSolver(algebraicModel[1])
            };

            var problem = new[] {
                new ProblemConvectionDiffusion(model[0], algebraicModel[0], solver[0]),
                new ProblemConvectionDiffusion(model[1], algebraicModel[1], solver[1])
            };

            var linearAnalyzer = new[] {
                new LinearAnalyzer(algebraicModel[0], solver[0], problem[0]),
                new LinearAnalyzer(algebraicModel[1], solver[1], problem[1])
            };

            var analyzer = new[] {
                new StaticAnalyzer(model[0], algebraicModel[0], solver[0], problem[0], linearAnalyzer[0]),
                new StaticAnalyzer(model[1], algebraicModel[1], solver[1], problem[1], linearAnalyzer[1])
            };

            watchDofs = new[] {
                new List<(INode node, IDofType dof)>(){ (model[0].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), },
                new List<(INode node, IDofType dof)>(){ (model[1].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), }
            };

            linearAnalyzer[0].LogFactory = new LinearAnalyzerLogFactory(watchDofs[0], algebraicModel[0]);
            linearAnalyzer[1].LogFactory = new LinearAnalyzerLogFactory(watchDofs[1], algebraicModel[1]);

            var staggeredAnalyzer = new StaggeredAnalyzer(analyzer, solver, CreateNewModel, maxStaggeredSteps: 1000, tolerance: 1e-5);
            staggeredAnalyzer.Initialize();
            //for (int i = 0; i < 1000 / 0.5; i++)
            //{
            //	staggeredAnalyzer.SolveCurrentStep();
            //	dynamicAnalyzer.AdvanceStep();
            //}
            staggeredAnalyzer.Solve();



            //DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            //var numericalSolution = new double[watchDofs.Count];
            //for (int i = 0; i < numericalSolution.Length; i++)
            //{
            //    numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            //}
            //Comsol3DStaggeredStSt.CheckResults(numericalSolution);



        }

        private static void CreateNewModel(IParentAnalyzer[] analyzers, ISolver[] solvers)
        {
            double u1 = ((DOFSLog)analyzers[0].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;
            double u2 = ((DOFSLog)analyzers[1].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;

            model = new[] {
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Hexa.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff:1d + u2),
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Hexa.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 2d, DependentProductionCoeff: 0d, IndependentProductionCoeff:2d + u1)
            };

            var solverFactory = new DenseMatrixSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
            var algebraicModel = new[] {
                solverFactory.BuildAlgebraicModel(model[0]),
                solverFactory.BuildAlgebraicModel(model[1])
            };

            solvers[0] = solverFactory.BuildSolver(algebraicModel[0]);
            solvers[1] = solverFactory.BuildSolver(algebraicModel[1]);

            var problem = new[] {
                new ProblemConvectionDiffusion(model[0], algebraicModel[0], solvers[0]),
                new ProblemConvectionDiffusion(model[1], algebraicModel[1], solvers[1])
            };

            linearAnalyzers = new[] {
                new LinearAnalyzer(algebraicModel[0], solvers[0], problem[0]),
                new LinearAnalyzer(algebraicModel[1], solvers[1], problem[1])
            };

            analyzers[0] = new StaticAnalyzer(model[0], algebraicModel[0], solvers[0], problem[0], linearAnalyzers[0]);
            analyzers[1] = new StaticAnalyzer(model[1], algebraicModel[1], solvers[1], problem[1], linearAnalyzers[1]);

            watchDofs = new[] {
                new List<(INode node, IDofType dof)>(){ (model[0].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), },
                new List<(INode node, IDofType dof)>(){ (model[1].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), }
            };

            linearAnalyzers[0].LogFactory = new LinearAnalyzerLogFactory(watchDofs[0], algebraicModel[0]);
            linearAnalyzers[1].LogFactory = new LinearAnalyzerLogFactory(watchDofs[1], algebraicModel[1]);

            analyzers[0].Initialize(true);
            analyzers[1].Initialize(true);
        }


        static void Comsol3DStaggeredDynamic()
        {
            model = new[] {
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Prism.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d, Capacity: 1d),
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Prism.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d, Capacity: 1d)
            };

/*            model = new[] {
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprus.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d, Capacity: 1d),
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprus.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d, Capacity: 1d)
            };*/

            //model = new[] {
            //    Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprusSparse.mphtxt", ConvectionCoeff: new double[] {0d, 0d, 0d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d, Capacity: 1d),
            //    Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprusSparse.mphtxt", ConvectionCoeff: new double[] {0d, 0d, 0d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d, Capacity: 1d)
            //};

            analyzerStates = new GenericAnalyzerState[model.Length];

            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = false }; //Dense Matrix Solver solves with zero matrices!
            //var solverFactory = new SkylineSolver.Factory(); //Dense Matrix Solver solves with zero matrices!

            var algebraicModel = new[] {
                solverFactory.BuildAlgebraicModel(model[0]),
                solverFactory.BuildAlgebraicModel(model[1])
            };

            var solver = new[] {
                solverFactory.BuildSolver(algebraicModel[0]),
                solverFactory.BuildSolver(algebraicModel[1])
            };

            var problem = new[] {
                new ProblemConvectionDiffusion(model[0], algebraicModel[0], solver[0]),
                new ProblemConvectionDiffusion(model[1], algebraicModel[1], solver[1])
            };

            linearAnalyzers = new[] {
                new LinearAnalyzer(algebraicModel[0], solver[0], problem[0]),
                new LinearAnalyzer(algebraicModel[1], solver[1], problem[1])
            };

            var analyzer = new IStepwiseAnalyzer[2];

            if (useNewmark)
            {
                analyzer[0] = (new NewmarkDynamicAnalyzer.Builder(model[0], algebraicModel[0], solver[0], problem[0], linearAnalyzers[0], timeStep: timeStep, totalTime: totalTime)).Build();
                analyzer[1] = (new NewmarkDynamicAnalyzer.Builder(model[1], algebraicModel[1], solver[1], problem[1], linearAnalyzers[1], timeStep: timeStep, totalTime: totalTime)).Build();
            }
            else
            {
                analyzer[0] = (new BDFDynamicAnalyzer.Builder(model[0], algebraicModel[0], solver[0], problem[0], linearAnalyzers[0], timeStep: timeStep, totalTime: totalTime, bdfOrder: bdford)).Build();
                analyzer[1] = (new BDFDynamicAnalyzer.Builder(model[1], algebraicModel[1], solver[1], problem[1], linearAnalyzers[1], timeStep: timeStep, totalTime: totalTime, bdfOrder: bdford)).Build();
            }

            /*            watchDofs = new[] {
                            new List<(INode node, IDofType dof)>(){ (model[0].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), },
                            new List<(INode node, IDofType dof)>(){ (model[1].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), }
                        };*/


            //Sparse Mesh
            watchDofs = new[] {
                new List<(INode node, IDofType dof)>(){ (model[0].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), },
                new List<(INode node, IDofType dof)>(){ (model[1].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), }
            };

            linearAnalyzers[0].LogFactory = new LinearAnalyzerLogFactory(watchDofs[0], algebraicModel[0]);
            linearAnalyzers[1].LogFactory = new LinearAnalyzerLogFactory(watchDofs[1], algebraicModel[1]);

            var u1s = new double[(int)(totalTime / timeStep)];
            var u2s = new double[(int)(totalTime / timeStep)];
            var staggeredAnalyzer = new StepwiseStaggeredAnalyzer(analyzer, solver, CreateNewModelDynamic, maxStaggeredSteps: maxStaggeredSteps, tolerance: 1e-5);
            staggeredAnalyzer.Initialize();
            for (currentTimeStep = 0; currentTimeStep < totalTime / timeStep; currentTimeStep++)
            {
                staggeredAnalyzer.SolveCurrentStep();
                u1s[currentTimeStep] = ((DOFSLog)analyzer[0].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;
                u2s[currentTimeStep] = ((DOFSLog)analyzer[1].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;

                for (int j = 0; j < analyzer.Length; j++)
                {
                    analyzer[j].AdvanceStep();
                }

                analyzerStates[0] = (analyzer[0] as IParentAnalyzer).CreateState();
                analyzerStates[1] = (analyzer[1] as IParentAnalyzer).CreateState();
            }

            Console.WriteLine("----- Final Solutions -----");
            for (int i = 0; i < u1s.Length; i++)
            {
                Console.WriteLine("-step: {0}\tu1 = {1}\tu2 = {2}", i, u1s[i].ToString("F5"), u2s[i].ToString("F5"));
            }
        }

        private static void CreateNewModelDynamic(IParentAnalyzer[] analyzers, ISolver[] solvers)
        {
            double u1 = ((DOFSLog)analyzers[0].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;
            double u2 = ((DOFSLog)analyzers[1].ChildAnalyzer.Logs[0]).DOFValues.FirstOrDefault().val;

/*            model = new[] {
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprusSparse.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d + u2, Capacity: 1d),
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprusSparse.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d + u1, Capacity: 1d)
            };*/
            //model = new[] {
            //    Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprusSparse.mphtxt", ConvectionCoeff: new double[] {0d, 0d, 0d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d + u2, Capacity: 1d),
            //    Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/OfficialMeshCyprusSparse.mphtxt", ConvectionCoeff: new double[] {0d, 0d, 0d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d + u1, Capacity: 1d)
            //};

            model = new[] {
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Prism.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d + u2, Capacity: 1d),
                Comsol3DStaggeredStSt.CreateModelFromComsolFile("../../../Meshes/3d8Prism.mphtxt", ConvectionCoeff: new double[] {1d, 1d, 1d}, DiffusionCoeff: 1d, DependentProductionCoeff: 0d, IndependentProductionCoeff: 1d + u1, Capacity: 1d)
            };

            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite=false };
            //var solverFactory = new SkylineSolver.Factory();

            var algebraicModel = new[] {
                solverFactory.BuildAlgebraicModel(model[0]),
                solverFactory.BuildAlgebraicModel(model[1])
            };

            solvers[0] = solverFactory.BuildSolver(algebraicModel[0]);
            solvers[1] = solverFactory.BuildSolver(algebraicModel[1]);

            var problem = new[] {
                new ProblemConvectionDiffusion(model[0], algebraicModel[0], solvers[0]),
                new ProblemConvectionDiffusion(model[1], algebraicModel[1], solvers[1])
            };

            var linearAnalyzer = new[] {
                new LinearAnalyzer(algebraicModel[0], solvers[0], problem[0]),
                new LinearAnalyzer(algebraicModel[1], solvers[1], problem[1])
            };

            var oldAnalyzers = analyzers.ToArray();

            if (useNewmark)
            {
                analyzers[0] = (new NewmarkDynamicAnalyzer.Builder(model[0], algebraicModel[0], solvers[0], problem[0], linearAnalyzer[0], timeStep: timeStep, totalTime: totalTime, currentStep: currentTimeStep)).Build();
                analyzers[1] = (new NewmarkDynamicAnalyzer.Builder(model[1], algebraicModel[1], solvers[1], problem[1], linearAnalyzer[1], timeStep: timeStep, totalTime: totalTime, currentStep: currentTimeStep)).Build();
            }
            else
            {
                analyzers[0] = (new BDFDynamicAnalyzer.Builder(model[0], algebraicModel[0], solvers[0], problem[0], linearAnalyzer[0], timeStep: timeStep, totalTime: totalTime, bdfOrder: bdford, currentTimeStep: currentTimeStep)).Build();
                analyzers[1] = (new BDFDynamicAnalyzer.Builder(model[1], algebraicModel[1], solvers[1], problem[1], linearAnalyzer[1], timeStep: timeStep, totalTime: totalTime, bdfOrder: bdford, currentTimeStep: currentTimeStep)).Build();
            }

/*            watchDofs = new[] {
                new List<(INode node, IDofType dof)>(){ (model[0].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), },
                n*//*ew List<(INode node, IDofType dof)>(){ (model[1].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), }
            };*/

            //Sparse Mesh
            watchDofs = new[] {
                new List<(INode node, IDofType dof)>(){ (model[0].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), },
                new List<(INode node, IDofType dof)>(){ (model[1].NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable), }
            };

            linearAnalyzer[0].LogFactory = new LinearAnalyzerLogFactory(watchDofs[0], algebraicModel[0]);
            linearAnalyzer[1].LogFactory = new LinearAnalyzerLogFactory(watchDofs[1], algebraicModel[1]);

            analyzers[0].Initialize(true);
            if (analyzerStates[0] != null)
            {
                analyzers[0].CurrentState = analyzerStates[0];
            }

            analyzers[1].Initialize(true);
            if (analyzerStates[1] != null)
            {
                analyzers[1].CurrentState = analyzerStates[1];
            }
        }

        static void Comsol3DConvectionDiffusionStadyStateHexaTest()
        {
            var model = Comsol3DConvectionDiffusionStadyStateHexa.CreateModel();
            //var solverFactory = new SkylineSolver.Factory(); //Dense Matrix Solver solves with zero matrices!
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

        static void Comsol3DConvectionDiffusionProductionDynamicHexaTest()
        {
            var model = Comsol3DConvectionDiffusionProductionDynamicHexa.CreateModel();
            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = false }; //Dense Matrix Solver solves with zero matrices!
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
            Comsol3DConvectionDiffusionProductionDynamicHexa.CheckResults(numericalSolution);
        }

    }
}
