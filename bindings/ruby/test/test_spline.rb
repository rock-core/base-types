require 'minitest/autorun'
require 'minitest/spec'
require 'sisl/spline'
require 'flexmock/minitest'

module SISL
    describe SISL::Spline do
        def test_base
            v = SISL::Spline.new(3)
            assert_equal 3, v.dimension
            assert_equal 0.1, v.geometric_resolution
            assert_equal 3, v.order

            v.interpolate([0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1])
            assert_equal [0, 0, 0], v.get(v.start_param)
            assert_equal [1, 1, 1], v.get(v.end_param)
        end

        def test_deriv
            v = SISL::Spline.new(3)

            v.interpolate([
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [1, 0, 0],
                [1, 0, 0],
                [1, 1, 0],
                [1, 1, 0]
            ], [], %I[
                ORDINARY_POINT
                TANGENT_POINT_FOR_NEXT
                TANGENT_POINT_FOR_PRIOR
                ORDINARY_POINT
                TANGENT_POINT_FOR_NEXT
                TANGENT_POINT_FOR_PRIOR
                ORDINARY_POINT
            ])

            assert_equal [0, 0, 0], v.get(v.start_param)
            assert_equal [1, 1, 0], v.get(v.end_param)
            assert_equal([0.75, 0, 0], v.get(0.5).map { |p| p.round(6) })
            assert_equal [1, 0.375, 0], v.get(1.5)
        end

        def test_concat
            v1 = SISL::Spline.new(3)
            v1.interpolate([0, 0, 0, 1, 2, 3])

            v2 = SISL::Spline.new(3)
            v2.interpolate([2, 2, 2, 4, 6, 8])

            result = v1.dup
            result.append(v2)
            assert_equal(v1.start_param, result.start_param)
            assert_equal(v2.end_param - v2.start_param + v1.end_param, result.end_param)
            assert_equal([0, 0, 0], result.start_point)
            assert_equal([3, 6, 9], result.end_point)
        end

        def test_join_singletons
            v1 = SISL::Spline.new(3)
            v1.interpolate([0, 0, 0])
            v2 = SISL::Spline.new(3)
            v2.interpolate([4, 4, 4])

            expected = SISL::Spline.new(3)
            expected.interpolate([0, 0, 0, 4, 4, 4])

            result = v1.dup
            result.join(v2, -1)
            assert_equal(expected.sample(0.1), result.sample(0.1))
        end

        def test_join_curve_and_singleton
            v1 = SISL::Spline.new(3)
            v1.interpolate([0, 0, 0])
            v2 = SISL::Spline.new(3)
            v2.interpolate([4, 4, 4, 6, 6, 6])

            result = v1.dup
            result.join(v2, -1)
            # TODO: find something to actually test ...
        end

        def test_join_singleton_and_curve
            v1 = SISL::Spline.new(3)
            v1.interpolate([0, 0, 0, 4, 4, 4])
            v2 = SISL::Spline.new(3)
            v2.interpolate([6, 6, 6])

            result = v1.dup
            result.join(v2, -1)
            # TODO: find something to actually test ...
        end

        def test_join
            v1 = SISL::Spline.new(3)
            v1.interpolate([0, 0, 0, 1, 2, 3])

            v2 = SISL::Spline.new(3)
            v2.interpolate([4, 4, 4, 4, 6, 8])

            result = v1.dup
            result.join(v2, -1)
            assert_equal(v1.start_param, result.start_param)
            assert_equal([0, 0, 0], result.start_point)
            assert_equal([4, 6, 8], result.end_point)
        end

        def test_distance_to
            v1 = SISL::Spline3.new
            v1.interpolate([Eigen::Vector3.new(0, 0, 0), Eigen::Vector3.new(1, 1, 1)])

            assert(v1.distance_to(Eigen::Vector3.new(0.5, 0.5, 0.5), 0, 1e-9) < 1e-9)
        end

        def test_find_one_closest_point
            v1 = SISL::Spline3.new
            v1.interpolate([Eigen::Vector3.new(0, 0, 0), Eigen::Vector3.new(1, 1, 1)])

            test_p = Eigen::Vector3.new(0.5, 0.5, 0.5)
            t = v1.find_one_closest_point(test_p, 0, 1e-9)
            assert v1.get(t).approx?(test_p)
        end

        describe 'the interpolator fluid interface' do
            before do
                @spline = Spline.new(2)
                flexmock(@spline).should_receive(:interpolate).
                    with_no_args.pass_thru
                flexmock(@spline).should_receive(:interpolate).
                    with_any_args.
                    and_return { |*args| @interpolation_args = args }
            end

            it 'allows to create an interpolation with only points' do
                @spline
                    .interpolate
                    .point(1, 2)
                    .knuckle_point(2, 3)
                    .point(3, 4)
                    .to_spline
                assert_interpolation_args [1, 2, 2, 3, 3, 4], [],
                                          %I[ORDINARY_POINT KNUCKLE_POINT ORDINARY_POINT]
            end

            it 'allows to pass parameters explicitely' do
                @spline
                    .interpolate
                    .at(0.1).point(1, 2)
                    .at(0.2).knuckle_point(2, 3)
                    .at(0.3).point(3, 4)
                    .to_spline
                assert_interpolation_args [1, 2, 2, 3, 3, 4], [0.1, 0.2, 0.3],
                                          %I[ORDINARY_POINT KNUCKLE_POINT ORDINARY_POINT]
            end

            it 'raises if a parameter is given after the first points' do
                e = assert_raises(ArgumentError) do
                    @spline
                        .interpolate
                        .point(1, 2)
                        .at(0.1).knuckle_point(2, 3)
                end
                assert_equal 'when used, at() must be called exactly once '\
                             'before each point', e.message
            end

            it 'raises if missing a parameter before a point' do
                e = assert_raises(ArgumentError) do
                    @spline
                        .interpolate
                        .at(0.1).point(1, 2)
                        .knuckle_point(2, 3)
                end
                assert_equal 'when used, at() must be called exactly once '\
                             'before each point', e.message
            end

            it 'raises if parameters are given without points' do
                e = assert_raises(ArgumentError) do
                    @spline
                        .interpolate
                        .at(0.1).at(0.2)
                        .knuckle_point(2, 3)
                end
                assert_equal 'when used, at() must be called exactly once '\
                             'before each point', e.message
            end

            it 'allows adding a derivative to prior' do
                @spline
                    .interpolate
                    .point(0, 1).derivative(1, 2)
                    .knuckle_point(2, 3)
                    .to_spline

                assert_interpolation_args [0, 1, 1, 2, 2, 3], [],
                                          %I[ORDINARY_POINT
                                             DERIVATIVE_TO_PRIOR
                                             KNUCKLE_POINT]
            end

            it 'raises if the derivative is given before a point' do
                e = assert_raises(ArgumentError) do
                    @spline
                        .interpolate
                        .derivative(0, 1).to_prior(1, 2)
                        .knuckle_point(2, 3)
                end
                assert_equal 'must define a point before calling #derivative',
                             e.message
            end

            it 'is usable from the class method' do
                flexmock(Spline).new_instances.
                    should_receive(:interpolate).with_no_args.never.pass_thru
                flexmock(Spline)
                    .new_instances.should_receive(:interpolate)
                    .with([0, 1, -1, -2, 2, 3], [], any)
                    .once.pass_thru

                spline = Spline
                         .interpolate(dimension: 2)
                         .point(0, 1)
                         .derivative(-1, -2)
                         .knuckle_point(2, 3)
                         .to_spline
                assert_kind_of Spline, spline
            end

            def assert_interpolation_args(points, parameters, types)
                assert_equal points, @interpolation_args[0], 'points'
                assert_equal parameters, @interpolation_args[1], 'parameters'
                assert_equal types, @interpolation_args[2], 'types'
            end
        end
    end
end
