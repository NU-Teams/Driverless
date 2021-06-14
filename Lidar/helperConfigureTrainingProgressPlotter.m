function lossPlotter = helperConfigureTrainingProgressPlotter(f)
% This function configures training progress plots for various losses.
    figure(f);
    clf
    ylabel('Total Loss');
    xlabel('Iteration');
    lossPlotter = animatedline;
end
