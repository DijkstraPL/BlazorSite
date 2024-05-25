using BeamStatica.Loads.PointLoads;
using BeamStatica.Results.Interfaces;
using BeamStatica.Results.Reactions;
using BeamStatica._spans;
using System;
using System.Collections.Generic;
using System.Linq;
using BendingMoment = BeamStatica.Loads.PointLoads.BendingMoment;

namespace BeamStatica.Results.OnSpan
{
    public class BendingMomentResult : IGetResult
    {
        public IResultValue Result { get; private set; }
        private IList<Span> _spans { get; }
        private double _currentLength;

        public BendingMomentResult(IList<Span> spans)
        {
            _spans = spans ?? throw new ArgumentNullException(nameof(spans));
        }

        public IResultValue GetValue(double distanceFromLeftSide)
        {
            Result = new Reactions.BendingMoment() { Value = 0 };
            _currentLength = 0;
            CalculateBendingMoment(distanceFromLeftSide);

            return Result;
        }

        private void CalculateBendingMoment(double distanceFromLeftSide)
        {
            foreach (var span in _spans)
            {
                CalculateBendingMomentFromNodeForces(distanceFromLeftSide, span);
                CalculateBendingMomentFromContinousLoads(distanceFromLeftSide, span);
                CalculateBendingMomentFromPointLoads(distanceFromLeftSide, span);

                _currentLength += span.Length;
                if (distanceFromLeftSide < _currentLength)
                    break;
            }
        }

        private void CalculateBendingMomentFromNodeForces(double distanceFromLeftSide, Span span)
        {
            Result.Value += span.LeftNode.BendingMoment?.Value ?? 0;
            Result.Value += (span.LeftNode.ShearForce?.Value ?? 0) * (distanceFromLeftSide - _currentLength);
            Result.Value += span.LeftNode.ConcentratedForces.Sum(l => l.CalculateBendingMoment(distanceFromLeftSide - _currentLength));
        }

        private void CalculateBendingMomentFromContinousLoads(double distanceFromLeftSide, Span span)
        {
            foreach (var load in span.ContinousLoads)
                Result.Value += load.CalculateBendingMoment(distanceFromLeftSide - load.StartPosition.Position - _currentLength);            
        }
        
        private void CalculateBendingMomentFromPointLoads(double distanceFromLeftSide, Span span)
        {
            foreach (var load in span.PointLoads)
                if (distanceFromLeftSide > load.Position + _currentLength)                    
                    Result.Value += load.CalculateBendingMoment(distanceFromLeftSide - load.Position - _currentLength);            
        }     
    }
}
