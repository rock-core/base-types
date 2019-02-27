require 'vizkit'
require 'minitest/spec'
require 'minitest/autorun'

module Vizkit
    module TestHelpers
        def setup
            super
            @timers = []
            @widgets = []
            Orocos.load unless Orocos.loaded?

            @confirmation_dialog = Qt::Widget.new
            layout = Qt::VBoxLayout.new(@confirmation_dialog)
            @text_field = Qt::Label.new
            layout.add_widget @text_field
            button_layout = Qt::HBoxLayout.new
            layout.add_layout button_layout
            button_layout.add_widget(@yes_btn = Qt::PushButton.new("Yes"))
            button_layout.add_widget(@no_btn = Qt::PushButton.new("No"))
            @yes_btn.connect(SIGNAL('clicked()')) do
                @confirm_result = true
            end
            @no_btn.connect(SIGNAL('clicked()')) do
                @confirm_result = false
            end
        end

        def teardown
            @confirmation_dialog.hide
            @timers.each(&:stop)
            @widgets.each(&:close)
            super
        end

        def timer(period)
            timer = Qt::Timer.new
            timer.connect(SIGNAL('timeout()')) do
                yield
            end
            timer.start(period)
            register_timer(timer)
        end

        def register_timer(timer)
            @timers << timer
        end

        def register_widget(widget)
            @widgets << widget
        end

        def run_confirmation_dialog(text, no_btn_visible: true)
            @text_field.text = text
            @confirm_result = nil
            if no_btn_visible
                @no_btn.show
            else
                @no_btn.hide
            end

            @confirmation_dialog.show
            while @confirm_result.nil?
                Vizkit.step
                sleep 0.01
            end
            @confirm_result
        ensure
            @confirmation_dialog.hide
        end

        def step(text)
            run_confirmation_dialog(text, no_btn_visible: false)
        end

        def confirm(text)
            valid = run_confirmation_dialog(text, no_btn_visible: true)
            assert valid, "#{text} was not confirmed by the user"
        end
    end
end

