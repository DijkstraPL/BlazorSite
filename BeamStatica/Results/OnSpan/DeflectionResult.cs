using BeamStatica._spans;
using BeamStatica.Loads.PointLoads;
using BeamStatica.Results.Displacements;
using BeamStatica.Results.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;

namespace BeamStatica.Results.OnSpan
{
    class DeflectionResult : IGetResult
    {
        public IResultValue Result { get; private set; }
        private IList<Span> _spans { get; }
        private double _currentLength;
        private double[] _spanDisplacements;

        public DeflectionResult(IList<Span> spans)
        {
            _spans = spans ?? throw new ArgumentNullException(nameof(spans));
        }

        public IResultValue GetValue(double distanceFromLeftSide)
        {
            Result = new Deflection() { Value = 0 };
            _currentLength = 0;
            _spanDisplacements = new double[_spans.Count];

            Result.Value += _spans.First().Displacements[0];
            Result.Value += _spans.First().Displacements[1] * (distanceFromLeftSide - _currentLength);

            int counter = 0;
            foreach (var span in _spans)
            {
                CalculateDeflectionFromNodeForces(distanceFromLeftSide, counter, span);
                CalculateDeflectionFromContinousLoads(distanceFromLeftSide, counter, span);
                CalculateDeflectionFromPointLoads(distanceFromLeftSide, counter, span);

                _spanDisplacements[counter] = _spanDisplacements[counter] / span.Material.YoungModulus / span.Section.MomentOfInteria * 100;

                _currentLength += span.Length;
                if (distanceFromLeftSide < _currentLength)
                    break;
                counter++;
            }

            Result.Value += _spanDisplacements.Sum();
            Result.Value *= 1000;
            return Result;

        }

        private void CalculateDeflectionFromNodeForces(double distanceFromLeftSide, int counter, Span span)
        {
            CalculateDeflectionFromMomentForces(distanceFromLeftSide, counter, span);
            CalculateDeflectionFromShearForces(distanceFromLeftSide, counter, span);
        }

        private void CalculateDeflectionFromContinousLoads(double distanceFromLeftSide, int counter, Span span)
        {
            foreach (var load in span.ContinousLoads)
            {
                if (distanceFromLeftSide > load.EndPosition.Position + _currentLength)
                    CalculateDeflectionOutsideLoadLength(distanceFromLeftSide, counter, load);
                else if (distanceFromLeftSide > load.StartPosition.Position + _currentLength)
                    CalculateDeflectionInsideLoadLength(distanceFromLeftSide, counter, load);
            }
        }

        private void CalculateDeflectionFromPointLoads(double distanceFromLeftSide, int counter, Span span)
        {
            foreach (var load in span.PointLoads)
            {
                if (distanceFromLeftSide > load.Position + _currentLength)
                    _spanDisplacements[counter] += load.Value * (distanceFromLeftSide - load.Position - _currentLength) * (distanceFromLeftSide - load.Position - _currentLength) * (distanceFromLeftSide - load.Position - _currentLength) / 2 / 3;
            }
        }

        private void CalculateDeflectionFromMomentForces(double distanceFromLeftSide, int counter, Span span)
        {
            _spanDisplacements[counter] += span.LeftNode.BendingMoment?.Value *
            (distanceFromLeftSide - _currentLength) *
            (distanceFromLeftSide - _currentLength)
            / 2 ?? 0;

            _spanDisplacements[counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is BendingMoment).Sum(cf => cf.Value) *
            (distanceFromLeftSide - _currentLength) *
            (distanceFromLeftSide - _currentLength)
            / 2;
        }

        private void CalculateDeflectionFromShearForces(double distanceFromLeftSide, int counter, Span span)
        {
            _spanDisplacements[counter] += span.LeftNode.ShearForce?.Value *
            (distanceFromLeftSide - _currentLength) *
            (distanceFromLeftSide - _currentLength) *
            (distanceFromLeftSide - _currentLength)
            / 2 / 3 ?? 0;

            _spanDisplacements[counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is ShearLoad).Sum(cf => cf.Value) *
            (distanceFromLeftSide - _currentLength) *
            (distanceFromLeftSide - _currentLength) *
            (distanceFromLeftSide - _currentLength)
            / 2 / 3;
        }

        private void CalculateDeflectionOutsideLoadLength(double distanceFromLeftSide, int counter, Loads.ContinousLoads.ContinousLoad load)
        {
            double forceAtX = GetForceAtTheCalculatedPoint(distanceFromLeftSide, load);

            _spanDisplacements[counter] += load.StartPosition.Value *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 4 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) * 2 / 3;
            _spanDisplacements[counter] += forceAtX *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 4 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3;

            _spanDisplacements[counter] -= load.EndPosition.Value *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 2 *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 3 *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 4 *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) * 2 / 3;
            _spanDisplacements[counter] -= forceAtX *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 2 *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 3 *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 4 *
                (distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 3;
        }

        private void CalculateDeflectionInsideLoadLength(double distanceFromLeftSide, int counter, Loads.ContinousLoads.ContinousLoad load)
        {
            double forceAtX = GetForceAtTheCalculatedPoint(distanceFromLeftSide, load);

            _spanDisplacements[counter] += load.StartPosition.Value *
               (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
               (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
               (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 4 *
               (distanceFromLeftSide - _currentLength - load.StartPosition.Position) * 2 / 3;
            _spanDisplacements[counter] += forceAtX *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 4 *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3;

        }

        private double GetForceAtTheCalculatedPoint(double distanceFromLeftSide, Loads.ContinousLoads.ContinousLoad load) 
            => (load.EndPosition.Value - load.StartPosition.Value) /
                (load.EndPosition.Position - load.StartPosition.Position) *
                (distanceFromLeftSide - _currentLength - load.StartPosition.Position) +
                load.StartPosition.Value;
    }
}
