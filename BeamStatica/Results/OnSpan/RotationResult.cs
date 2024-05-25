using BeamStatica._spans;
using BeamStatica.Loads.PointLoads;
using BeamStatica.Results.Displacements;
using BeamStatica.Results.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;

namespace BeamStatica.Results.OnSpan
{
    public class RotationResult : IGetResult
    {
        public IResultValue Result { get; private set; }
        private readonly IList<Span> _spans;
        private double _currentLength;
        private double[] _spanRotation;
        private int _counter = 0;
        private double _distanceFromLeftSide;

        public RotationResult(IList<Span> spans)
        {
            _spans = spans ?? throw new ArgumentNullException(nameof(spans));
        }

        public IResultValue GetValue(double distanceFromLeftSide)
        {
            _distanceFromLeftSide = distanceFromLeftSide;
            Result = new Rotation() { Value = 0 };

            _spanRotation = new double[_spans.Count];
            _currentLength = 0;
            Result.Value += _spans.First().Displacements[1];

            _counter = 0;
            CalculateRotation();

            Result.Value += _spanRotation.Sum(); // / avgYoungModulus / avgMomentOfInteria * 100;

            return Result;
        }

        private void CalculateRotation()
        {
            foreach (var span in _spans)
            {
                CalculateRotationFromNodeForces(span);
                CalculateRotationFromContinousLoads(span);
                CalculateRotationFromPointLoads(span);

                _spanRotation[_counter] = _spanRotation[_counter] * 100;
                //_spanRotation[_counter] = _spanRotation[_counter] / span.Material.YoungModulus / span.Section.MomentOfInteria * 100;

                _currentLength += span.Length;
                if (_distanceFromLeftSide < _currentLength)
                    break;
                _counter++;
            }
        }

        private void CalculateRotationFromNodeForces(Span span)
        {
            CalculateRotationFromMomentForces(span);
            CalculateRotationFromShearForces(span);
        }

        private void CalculateRotationFromContinousLoads(Span span)
        {
            foreach (var load in span.ContinousLoads)
            {
                if (_distanceFromLeftSide > load.EndPosition.Position + _currentLength)
                    CalculateRotationOutsideLoadLength(load);
                else if (_distanceFromLeftSide > load.StartPosition.Position + _currentLength)
                    CalculateRotationInsideLoadLength(load);
            }
        }

        private void CalculateRotationFromPointLoads(Span span)
        {
            foreach (var load in span.PointLoads)
            {
                if (_distanceFromLeftSide > load.Position + _currentLength)
                {
                    double calculatedSpanLengths = 0;
                    foreach (var singleSpan in _spans)
                    {
                        calculatedSpanLengths += singleSpan.Length;
                        if (span == singleSpan)
                        {
                            if (_distanceFromLeftSide <= calculatedSpanLengths)
                            {
                                _spanRotation[_counter] += load.Value
                                   * (_distanceFromLeftSide - load.Position - _currentLength)
                                   * (_distanceFromLeftSide - load.Position - _currentLength) / 2
                                   / singleSpan.Material.YoungModulus
                                   / singleSpan.Section.MomentOfInteria;

                                break;
                            }
                            else
                            {
                                _spanRotation[_counter] += load.Value
                                   * (singleSpan.Length - load.Position)
                                   * (singleSpan.Length - load.Position) / 2
                                   / singleSpan.Material.YoungModulus
                                   / singleSpan.Section.MomentOfInteria;

                                continue;
                            }
                        }
                        
                        if (_distanceFromLeftSide > calculatedSpanLengths)
                        {
                            _spanRotation[_counter] += load.Value
                                * singleSpan.Length
                                * singleSpan.Length / 2
                                / singleSpan.Material.YoungModulus
                                / singleSpan.Section.MomentOfInteria;                            
                        }
                        else
                        {
                            _spanRotation[_counter] += load.Value
                                * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length)
                                * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length) / 2
                                / singleSpan.Material.YoungModulus
                                / singleSpan.Section.MomentOfInteria;
                        }                        
                    }
                    //_spanRotation[_counter] += load.Value *
                    //(_distanceFromLeftSide - load.Position - _currentLength) *
                    //(_distanceFromLeftSide - load.Position - _currentLength)
                    /// 2;
                }
            }
        }

        private void CalculateRotationFromMomentForces(Span span)
        {
            double calculatedSpanLengths = 0;
            bool properSpan = false;
            foreach (var singleSpan in _spans)
            {
                calculatedSpanLengths += singleSpan.Length;
                if (span == singleSpan)
                    properSpan = true;

                if (!properSpan)
                    continue;

                if (_distanceFromLeftSide < calculatedSpanLengths)
                {
                    _spanRotation[_counter] += (span.LeftNode.BendingMoment?.Value ?? 0)
                        * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length)
                        / singleSpan.Material.YoungModulus
                        / singleSpan.Section.MomentOfInteria;

                    _spanRotation[_counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is BendingMoment).Sum(cf => cf.Value)
                        * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length)
                        / singleSpan.Material.YoungModulus
                        / singleSpan.Section.MomentOfInteria;

                    break;
                }

                _spanRotation[_counter] +=(span.LeftNode.BendingMoment?.Value ?? 0)
                    * singleSpan.Length
                    / singleSpan.Material.YoungModulus
                    / singleSpan.Section.MomentOfInteria;

                _spanRotation[_counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is BendingMoment).Sum(cf => cf.Value)
                    * singleSpan.Length
                    / singleSpan.Material.YoungModulus
                    / singleSpan.Section.MomentOfInteria;
            }

            //_spanRotation[_counter] += span.LeftNode.BendingMoment?.Value * (_distanceFromLeftSide - _currentLength) ?? 0;
            //_spanRotation[_counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is BendingMoment).Sum(cf => cf.Value) *
            //    (_distanceFromLeftSide - _currentLength);
        }

        private void CalculateRotationFromShearForces(Span span)
        {
            double calculatedSpanLengths = 0;
            bool properSpan = false;
            foreach (var singleSpan in _spans)
            {
                calculatedSpanLengths += singleSpan.Length;
                if (span == singleSpan)
                    properSpan = true;

                if (!properSpan)
                    continue;

                if (_distanceFromLeftSide < calculatedSpanLengths)
                {
                    _spanRotation[_counter] += (span.LeftNode.ShearForce?.Value
                        * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length)
                        * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length) / 2 ?? 0)
                        / singleSpan.Material.YoungModulus
                        / singleSpan.Section.MomentOfInteria;

                    _spanRotation[_counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is ShearLoad).Sum(cf => cf.Value)
                        * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length)
                        * (_distanceFromLeftSide - calculatedSpanLengths + singleSpan.Length) / 2
                        / singleSpan.Material.YoungModulus
                        / singleSpan.Section.MomentOfInteria;

                    break;
                }

                _spanRotation[_counter] += (span.LeftNode.ShearForce?.Value 
                    * singleSpan.Length 
                    * singleSpan.Length / 2 ?? 0)
                    / singleSpan.Material.YoungModulus
                    / singleSpan.Section.MomentOfInteria;

                _spanRotation[_counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is ShearLoad).Sum(cf => cf.Value)
                    * singleSpan.Length
                    * singleSpan.Length / 2
                    / singleSpan.Material.YoungModulus
                    / singleSpan.Section.MomentOfInteria;
            }




            //_spanRotation[_counter] += (span.LeftNode.ShearForce?.Value * (_distanceFromLeftSide - _currentLength) *
            //    (_distanceFromLeftSide - _currentLength) / 2 ?? 0);
            //_spanRotation[_counter] += span.LeftNode.ConcentratedForces.Where(cf => cf is ShearLoad).Sum(cf => cf.Value) *
            //    (_distanceFromLeftSide - _currentLength) * (_distanceFromLeftSide - _currentLength) / 2;
        }

        private void CalculateRotationOutsideLoadLength(Loads.ContinousLoads.ContinousLoad load)
        {
            double forceAtX = GetForceAtTheCalculatedPoint(load);

            _spanRotation[_counter] += load.StartPosition.Value *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) * 2 / 3;
            _spanRotation[_counter] += forceAtX *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3;

            _spanRotation[_counter] -= load.EndPosition.Value *
                (_distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 2 *
                (_distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 3 *
                (_distanceFromLeftSide - _currentLength - load.EndPosition.Position) * 2 / 3;
            _spanRotation[_counter] -= forceAtX *
                (_distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 2 *
                (_distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 3 *
                (_distanceFromLeftSide - _currentLength - load.EndPosition.Position) / 3;
        }

        private void CalculateRotationInsideLoadLength(Loads.ContinousLoads.ContinousLoad load)
        {
            double forceAtX = GetForceAtTheCalculatedPoint(load);

            _spanRotation[_counter] += load.StartPosition.Value *
               (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
               (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
               (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) * 2 / 3;
            _spanRotation[_counter] += forceAtX *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 2 *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3 *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) / 3;
        }

        private double GetForceAtTheCalculatedPoint(Loads.ContinousLoads.ContinousLoad load)
            => (load.EndPosition.Value - load.StartPosition.Value) /
                (load.EndPosition.Position - load.StartPosition.Position) *
                (_distanceFromLeftSide - _currentLength - load.StartPosition.Position) +
                load.StartPosition.Value;
    }
}
